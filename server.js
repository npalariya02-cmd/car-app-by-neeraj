'use strict';

const http = require('http');
const path = require('path');
const fs = require('fs');
const os = require('os');
const { spawn } = require('child_process');
const express = require('express');
const { Server } = require('socket.io');

const PORT = Number(process.env.PORT) || 3000;
const HOST = '0.0.0.0';
const PORT_REGEX = /^(COM\d{1,3}|\/dev\/[A-Za-z0-9._-]+)$/;

const ARDUINO_SKETCH = String.raw`#include <Servo.h>

// ---- Pinout (L298N, HC-SR04, SG90) ----
const uint8_t ENA = 5;
const uint8_t IN1 = 2;
const uint8_t IN2 = 3;
const uint8_t IN3 = 4;
const uint8_t IN4 = 7;
const uint8_t ENB = 6;

const uint8_t TRIG = 8;
const uint8_t ECHO = 9;
const uint8_t SERVO_PIN = 10;

const int AVOID_CM = 25;
const unsigned long DIST_INTERVAL_MS = 150;
const unsigned long BACKUP_MS = 350;

// Flip these if left or right wheels spin backwards
const bool SWAP_LEFT = false;
const bool SWAP_RIGHT = false;

Servo panServo;
int speedVal = 165;
char moving = 'S';
bool autoAvoid = false;
unsigned long lastDistMs = 0;
unsigned long backupUntilMs = 0;
int lastCm = 999;
String cmdBuf = "";

void setSide(uint8_t pinA, uint8_t pinB, int dir, bool swap) {
  if (swap) dir = -dir;
  digitalWrite(pinA, dir > 0 ? HIGH : LOW);
  digitalWrite(pinB, dir < 0 ? HIGH : LOW);
}

void drive(char c) {
  moving = c;
  int leftDir = 0;
  int rightDir = 0;
  if (c == 'F') { leftDir = 1;  rightDir = 1;  }
  if (c == 'B') { leftDir = -1; rightDir = -1; }
  if (c == 'L') { leftDir = -1; rightDir = 1;  }
  if (c == 'R') { leftDir = 1;  rightDir = -1; }

  setSide(IN1, IN2, leftDir, SWAP_LEFT);
  setSide(IN3, IN4, rightDir, SWAP_RIGHT);

  int pwm = (c == 'S') ? 0 : speedVal;
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

int readCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 25000UL);
  if (duration <= 0) return 999;
  return (int)(duration / 58);
}

void handleCmd(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s == "F" || s == "B" || s == "L" || s == "R" || s == "S") {
    if (s == "F" && autoAvoid && lastCm < AVOID_CM) {
      drive('S');
      return;
    }
    backupUntilMs = 0;
    drive(s.charAt(0));
  } else if (s == "O") {
    autoAvoid = true;
  } else if (s == "M") {
    autoAvoid = false;
    backupUntilMs = 0;
  } else if (s.startsWith("SPD:")) {
    speedVal = constrain(s.substring(4).toInt(), 70, 255);
    if (moving != 'S') {
      drive(moving);
    }
  } else if (s.startsWith("CAM:")) {
    int angle = constrain(s.substring(4).toInt(), 20, 160);
    panServo.write(angle);
  }
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  drive('S');
  panServo.attach(SERVO_PIN);
  panServo.write(90);

  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuf.length() > 0) {
        handleCmd(cmdBuf);
        cmdBuf = "";
      }
    } else if (cmdBuf.length() < 32) {
      cmdBuf += ch;
    }
  }

  unsigned long now = millis();

  if (backupUntilMs > 0 && now >= backupUntilMs) {
    backupUntilMs = 0;
    if (moving == 'B') {
      drive('S');
    }
  }

  if (now - lastDistMs >= DIST_INTERVAL_MS) {
    lastDistMs = now;
    lastCm = readCm();
    Serial.print("DIST:");
    Serial.println(lastCm);

    if (autoAvoid && moving == 'F' && lastCm < AVOID_CM) {
      drive('S');
      drive('B');
      backupUntilMs = now + BACKUP_MS;
    }
  }
}
`;

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST']
  }
});

app.use(express.json({ limit: '64kb' }));
app.use(express.static(path.join(__dirname, 'public')));

app.get('/', (req, res) => {
  res.redirect('/laptop-controller.html');
});

app.get('/api/arduino/sketch', (req, res) => {
  res.type('text/plain; charset=utf-8').send(ARDUINO_SKETCH);
});

function parseArduinoPorts(rawJson) {
  const data = JSON.parse(rawJson);
  const rawList = Array.isArray(data)
    ? data
    : (data && Array.isArray(data.detected_ports) ? data.detected_ports : []);

  const ports = [];
  for (const entry of rawList) {
    const portObj = entry && entry.port ? entry.port : entry;
    const address = portObj && typeof portObj.address === 'string' ? portObj.address : '';
    if (!address) continue;

    let boardName = '';
    if (entry && Array.isArray(entry.matching_boards) && entry.matching_boards.length > 0) {
      boardName = entry.matching_boards[0].name || '';
    } else if (entry && Array.isArray(entry.boards) && entry.boards.length > 0) {
      boardName = entry.boards[0].name || '';
    }

    const proto = portObj.protocol_label || portObj.label || '';
    const label = boardName
      ? `${boardName} (${address})`
      : (proto && proto !== address ? `${address} - ${proto}` : address);

    ports.push({ address, label });
  }
  return ports;
}

app.get('/api/arduino/ports', (req, res) => {
  let responded = false;
  let stdout = '';
  let stderr = '';

  const finish = (status, payload) => {
    if (responded) return;
    responded = true;
    res.status(status).json(payload);
  };

  let child;
  try {
    child = spawn('arduino-cli', ['board', 'list', '--format', 'json'], { shell: false });
  } catch (err) {
    return finish(200, {
      ok: false,
      cliInstalled: false,
      error: 'arduino-cli not found',
      ports: []
    });
  }

  child.on('error', (err) => {
    const missing = err && err.code === 'ENOENT';
    finish(200, {
      ok: false,
      cliInstalled: !missing,
      error: missing ? 'arduino-cli not found' : (err.message || 'Failed to launch arduino-cli'),
      ports: []
    });
  });

  child.stdout.on('data', (chunk) => {
    stdout += chunk.toString();
  });

  child.stderr.on('data', (chunk) => {
    stderr += chunk.toString();
  });

  child.on('close', (code) => {
    if (responded) return;
    if (code !== 0) {
      return finish(200, {
        ok: false,
        cliInstalled: true,
        error: stderr.trim() || `arduino-cli exited with code ${code}`,
        ports: []
      });
    }
    try {
      const ports = parseArduinoPorts(stdout);
      finish(200, { ok: true, cliInstalled: true, ports });
    } catch (err) {
      finish(200, {
        ok: false,
        cliInstalled: true,
        error: `Failed to parse arduino-cli output: ${err.message}`,
        ports: []
      });
    }
  });
});

let uploadInProgress = false;

function runCliCommand(args) {
  return new Promise((resolve) => {
    let log = '';
    let settled = false;

    const emitLine = (line) => {
      const clean = line.replace(/\r$/, '');
      if (!clean) return;
      log += clean + '\n';
      io.emit('arduino-log', clean);
    };

    const done = (result) => {
      if (settled) return;
      settled = true;
      resolve(result);
    };

    let child;
    try {
      child = spawn('arduino-cli', args, { shell: false });
    } catch (err) {
      emitLine(`ERROR: ${err.message}`);
      return done({ code: -1, log, missingCli: err.code === 'ENOENT' });
    }

    let outBuf = '';
    let errBuf = '';

    child.on('error', (err) => {
      const missing = err && err.code === 'ENOENT';
      emitLine(missing ? 'ERROR: arduino-cli not found on server PATH.' : `ERROR: ${err.message}`);
      done({ code: -1, log, missingCli: missing });
    });

    child.stdout.on('data', (chunk) => {
      outBuf += chunk.toString();
      const lines = outBuf.split('\n');
      outBuf = lines.pop();
      lines.forEach(emitLine);
    });

    child.stderr.on('data', (chunk) => {
      errBuf += chunk.toString();
      const lines = errBuf.split('\n');
      errBuf = lines.pop();
      lines.forEach(emitLine);
    });

    child.on('close', (code) => {
      if (outBuf) emitLine(outBuf);
      if (errBuf) emitLine(errBuf);
      done({ code: typeof code === 'number' ? code : -1, log, missingCli: false });
    });
  });
}

app.post('/api/arduino/upload', async (req, res) => {
  const port = req.body && typeof req.body.port === 'string' ? req.body.port.trim() : '';
  if (!PORT_REGEX.test(port)) {
    return res.status(400).json({
      ok: false,
      error: 'Invalid serial port. Expected COMx or /dev/...',
      log: ''
    });
  }

  if (uploadInProgress) {
    return res.status(409).json({
      ok: false,
      error: 'An Arduino upload is already in progress.',
      log: ''
    });
  }

  uploadInProgress = true;
  let combinedLog = '';

  const pushLog = (line) => {
    combinedLog += line + '\n';
    io.emit('arduino-log', line);
  };

  try {
    const sketchDir = path.join(os.tmpdir(), 'robocar');
    const sketchPath = path.join(sketchDir, 'robocar.ino');
    fs.mkdirSync(sketchDir, { recursive: true });
    fs.writeFileSync(sketchPath, ARDUINO_SKETCH, 'utf8');

    pushLog(`> Writing sketch to ${sketchPath}`);
    pushLog('> Compiling for arduino:avr:uno...');

    const compileRes = await runCliCommand(['compile', '--fqbn', 'arduino:avr:uno', sketchDir]);
    combinedLog += compileRes.log;

    if (compileRes.code !== 0) {
      if (/platform\s+'?arduino:avr'?\s+not found|core\s+not\s+found|missing.*arduino:avr/i.test(compileRes.log)) {
        pushLog('HINT: Run once: arduino-cli core update-index && arduino-cli core install arduino:avr');
      }
      return res.status(200).json({ ok: false, log: combinedLog });
    }

    pushLog(`> Uploading to ${port}...`);
    const uploadRes = await runCliCommand(['upload', '-p', port, '--fqbn', 'arduino:avr:uno', sketchDir]);
    combinedLog += uploadRes.log;

    if (uploadRes.code === 0) {
      pushLog('> Upload complete!');
      return res.status(200).json({ ok: true, log: combinedLog });
    }

    return res.status(200).json({ ok: false, log: combinedLog });
  } catch (err) {
    pushLog(`ERROR: ${err.message}`);
    return res.status(500).json({ ok: false, error: err.message, log: combinedLog });
  } finally {
    uploadInProgress = false;
  }
});

const roles = {
  laptop: null,
  phone: null
};

function emitPeerStatus() {
  io.emit('peer-status', {
    laptop: Boolean(roles.laptop),
    phone: Boolean(roles.phone)
  });
}

function getOtherRoleSocketId(socket) {
  const role = socket.data && socket.data.role
    ? socket.data.role
    : (roles.laptop === socket.id ? 'laptop' : (roles.phone === socket.id ? 'phone' : null));
  if (role === 'laptop') return roles.phone;
  if (role === 'phone') return roles.laptop;
  return null;
}

io.on('connection', (socket) => {
  emitPeerStatus();

  socket.on('register', (role) => {
    if (role !== 'laptop' && role !== 'phone') return;

    if (roles.laptop === socket.id) roles.laptop = null;
    if (roles.phone === socket.id) roles.phone = null;

    roles[role] = socket.id;
    socket.data.role = role;
    emitPeerStatus();

    if (role === 'laptop' && roles.phone) {
      io.to(roles.phone).emit('viewer-ready');
    } else if (role === 'phone' && roles.laptop) {
      io.to(roles.phone).emit('viewer-ready');
    }
  });

  socket.on('reconnect-viewer', () => {
    if (roles.phone) {
      io.to(roles.phone).emit('viewer-ready');
    }
  });

  socket.on('offer', (payload) => {
    const target = getOtherRoleSocketId(socket);
    if (target) io.to(target).emit('offer', payload);
  });

  socket.on('answer', (payload) => {
    const target = getOtherRoleSocketId(socket);
    if (target) io.to(target).emit('answer', payload);
  });

  socket.on('ice-candidate', (payload) => {
    const target = getOtherRoleSocketId(socket);
    if (target) io.to(target).emit('ice-candidate', payload);
  });

  socket.on('cmd', (c) => {
    if (typeof c === 'string' && c.length > 0 && c.length <= 16) {
      io.emit('cmd', c);
    }
  });

  socket.on('disconnect', () => {
    let changed = false;
    if (roles.laptop === socket.id) {
      roles.laptop = null;
      changed = true;
    }
    if (roles.phone === socket.id) {
      roles.phone = null;
      changed = true;
    }
    if (changed) {
      emitPeerStatus();
    }
  });
});

server.listen(PORT, HOST, () => {
  console.log(`GTA 5 RoboCar Bridge listening on http://${HOST}:${PORT}`);
});
