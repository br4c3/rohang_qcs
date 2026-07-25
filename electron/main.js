const {
  app,
  BrowserWindow,
  desktopCapturer,
  dialog,
  ipcMain,
  shell,
} = require("electron");
const { spawn } = require("node:child_process");
const http = require("node:http");
const path = require("node:path");
const fs = require("node:fs");

const projectRoot = path.join(__dirname, "..");
const swaggerAssets = path.join(projectRoot, "node_modules", "swagger-ui-dist");
const koreanFontAssets = path.join(
  projectRoot,
  "node_modules",
  "@fontsource",
  "noto-sans-kr",
);

const contentTypes = {
  ".css": "text/css; charset=utf-8",
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".woff": "font/woff",
  ".woff2": "font/woff2",
  ".yaml": "application/yaml; charset=utf-8",
  ".yml": "application/yaml; charset=utf-8",
};

let server;
let rosBridge;
let cameraBridge;
let qgcProcess;
let mavrosProcess;
let missionProcess;
let selectedPlanPath;
let qgcCaptureTimer;
let qgcCaptureBusy = false;
let qgcCaptureReady = false;
let qgcCaptureSourcesLogged = false;

function resolveRequestPath(requestUrl) {
  const url = new URL(requestUrl, "http://127.0.0.1");
  const decodedPath = decodeURIComponent(url.pathname);

  if (decodedPath.startsWith("/vendor/swagger-ui/")) {
    const fileName = decodedPath.slice("/vendor/swagger-ui/".length);
    return path.join(swaggerAssets, path.basename(fileName));
  }

  if (decodedPath.startsWith("/vendor/leaflet/")) {
    const fileName = decodedPath.slice("/vendor/leaflet/".length);
    const leafletAssets = path.join(projectRoot, "node_modules", "leaflet", "dist");
    return path.join(leafletAssets, path.basename(fileName));
  }

  if (decodedPath.startsWith("/vendor/noto-sans-kr/")) {
    const relativeFontPath = decodedPath.slice("/vendor/noto-sans-kr/".length);
    const requestedFontPath = path.resolve(koreanFontAssets, relativeFontPath);
    const relativeToFontRoot = path.relative(
      koreanFontAssets,
      requestedFontPath,
    );
    if (
      relativeToFontRoot.startsWith("..")
      || path.isAbsolute(relativeToFontRoot)
    ) {
      return null;
    }
    return requestedFontPath;
  }

  const relativePath = decodedPath === "/" ? "index.html" : decodedPath.slice(1);
  const requestedPath = path.resolve(projectRoot, relativePath);
  const relativeToRoot = path.relative(projectRoot, requestedPath);

  if (relativeToRoot.startsWith("..") || path.isAbsolute(relativeToRoot)) {
    return null;
  }

  return requestedPath;
}

function serveFile(request, response) {
  const filePath = resolveRequestPath(request.url);

  if (!filePath) {
    response.writeHead(403).end("Forbidden");
    return;
  }

  fs.stat(filePath, (statError, stat) => {
    if (statError || !stat.isFile()) {
      response.writeHead(404).end("Not found");
      return;
    }

    const contentType = contentTypes[path.extname(filePath)] || "application/octet-stream";
    response.writeHead(200, {
      "Content-Type": contentType,
      "Cache-Control": "no-store",
      "X-Content-Type-Options": "nosniff",
    });

    if (request.method === "HEAD") {
      response.end();
      return;
    }

    fs.createReadStream(filePath).pipe(response);
  });
}

function startServer() {
  return new Promise((resolve, reject) => {
    server = http.createServer(serveFile);
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      resolve(`http://127.0.0.1:${address.port}`);
    });
  });
}

function createWindow(baseUrl) {
  const window = new BrowserWindow({
    width: 1280,
    height: 760,
    minWidth: 920,
    minHeight: 620,
    backgroundColor: "#edf3f5",
    frame: false,
    autoHideMenuBar: true,
    title: "ARECADA GCS",
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
      preload: path.join(__dirname, "preload.js"),
    },
  });

  window.loadURL(baseUrl);
  const sendMaximizedState = () => {
    window.webContents.send("window:maximized", window.isMaximized());
  };
  window.on("maximize", sendMaximizedState);
  window.on("unmaximize", sendMaximizedState);

  window.webContents.setWindowOpenHandler(({ url }) => {
    if (!url.startsWith(baseUrl)) {
      shell.openExternal(url);
      return { action: "deny" };
    }

    return { action: "allow" };
  });
}

function windowFromEvent(event) {
  return BrowserWindow.fromWebContents(event.sender);
}

ipcMain.on("window:minimize", (event) => {
  windowFromEvent(event)?.minimize();
});

ipcMain.on("window:toggle-maximize", (event) => {
  const window = windowFromEvent(event);
  if (!window) return;
  if (window.isMaximized()) {
    window.unmaximize();
  } else {
    window.maximize();
  }
});

ipcMain.on("window:close", (event) => {
  windowFromEvent(event)?.close();
});

function startRosBridge() {
  const bridgePath = path.join(projectRoot, "bridge", "px4_bridge.py");

  rosBridge = spawn("python3", [bridgePath], {
    cwd: projectRoot,
    stdio: ["pipe", "pipe", "pipe"],
    env: {
      ...process.env,
      PYTHONUNBUFFERED: "1",
    },
  });

  let bufferedOutput = "";

  rosBridge.stdout.on("data", (chunk) => {
    bufferedOutput += chunk.toString();
    const lines = bufferedOutput.split("\n");
    bufferedOutput = lines.pop();

    for (const line of lines) {
      try {
        const message = JSON.parse(line);
        BrowserWindow.getAllWindows().forEach((window) => {
          window.webContents.send("px4:telemetry", message);
        });
      } catch {
        continue;
      }
    }
  });

  rosBridge.stderr.on("data", (chunk) => {
    console.error(`[PX4 bridge] ${chunk.toString().trim()}`);
  });

  rosBridge.on("error", (error) => {
    console.error(`[PX4 bridge] Failed to start: ${error.message}`);
  });
}

function startMavros() {
  mavrosProcess = spawn(
    "ros2",
    [
      "run",
      "mavros",
      "mavros_node",
      "--ros-args",
      "--params-file",
      "/opt/ros/humble/share/mavros/launch/px4_config.yaml",
      "-p",
      "fcu_url:=udp://:14540@127.0.0.1:14580",
    ],
    {
      cwd: projectRoot,
      stdio: ["ignore", "ignore", "pipe"],
      env: process.env,
    },
  );

  mavrosProcess.stderr.on("data", (chunk) => {
    const message = chunk.toString();
    if (message.includes("ERROR") || message.includes("FATAL")) {
      console.error(`[MAVROS] ${message.trim()}`);
    }
  });

  mavrosProcess.on("error", (error) => {
    console.error(`[MAVROS] Failed to start: ${error.message}`);
  });
}

function broadcast(channel, payload) {
  BrowserWindow.getAllWindows().forEach((window) => {
    window.webContents.send(channel, payload);
  });
}

function startCameraBridge() {
  const bridgePath = path.join(
    projectRoot,
    "bridge",
    "gazebo_camera_bridge.py",
  );

  cameraBridge = spawn("python3", [bridgePath], {
    cwd: projectRoot,
    stdio: ["ignore", "pipe", "pipe"],
    env: process.env,
  });

  let frameBuffer = Buffer.alloc(0);
  let cameraConnected = false;

  cameraBridge.stdout.on("data", (chunk) => {
    frameBuffer = Buffer.concat([frameBuffer, chunk]);

    while (frameBuffer.length >= 4) {
      const frameLength = frameBuffer.readUInt32BE(0);

      if (frameBuffer.length < frameLength + 4) {
        break;
      }

      const frame = frameBuffer.subarray(4, frameLength + 4);
      frameBuffer = frameBuffer.subarray(frameLength + 4);

      if (!cameraConnected) {
        cameraConnected = true;
        console.log(
          `[Gazebo camera] Streaming JPEG frames (${frame.length} bytes first frame)`,
        );
        broadcast("gazebo:camera-status", true);
      }

      broadcast("gazebo:camera-frame", frame);
    }
  });

  cameraBridge.stderr.on("data", (chunk) => {
    console.log(`[Gazebo camera] ${chunk.toString().trim()}`);
  });

  cameraBridge.on("error", (error) => {
    console.error(`[Gazebo camera] Failed to start: ${error.message}`);
    broadcast("gazebo:camera-status", false);
  });

  cameraBridge.on("exit", () => {
    broadcast("gazebo:camera-status", false);
  });
}

function qgcExecutablePath() {
  return process.env.QGC_PATH || "/home/br4c3/apps/QGroundControl.AppImage";
}

function launchQGroundControl() {
  if (qgcProcess && qgcProcess.exitCode === null) {
    broadcast("qgc:status", {
      status: "running",
      detail: "MAVLink UDP 14550",
    });
    return { ok: true, alreadyRunning: true };
  }

  const executable = qgcExecutablePath();

  if (!fs.existsSync(executable)) {
    const error = `QGroundControl not found: ${executable}`;
    broadcast("qgc:status", { status: "missing", detail: executable });
    return { ok: false, error };
  }

  broadcast("qgc:status", {
    status: "starting",
    detail: "Opening QGroundControl",
  });

  qgcProcess = spawn(executable, [], {
    cwd: path.dirname(executable),
    stdio: "ignore",
    env: process.env,
  });

  qgcProcess.once("spawn", () => {
    broadcast("qgc:status", {
      status: "running",
      detail: "MAVLink UDP 14550",
    });
  });

  qgcProcess.once("error", (error) => {
    broadcast("qgc:status", {
      status: "error",
      detail: error.message,
    });
  });

  qgcProcess.once("exit", () => {
    qgcProcess = null;
    broadcast("qgc:status", {
      status: "stopped",
      detail: "QGroundControl closed",
    });
  });

  return { ok: true, alreadyRunning: false };
}

function startQgcCapture() {
  if (qgcCaptureTimer) return;

  qgcCaptureTimer = setInterval(async () => {
    if (qgcCaptureBusy) {
      return;
    }

    qgcCaptureBusy = true;
    try {
      const sources = await desktopCapturer.getSources({
        types: ["window"],
        thumbnailSize: { width: 1280, height: 720 },
        fetchWindowIcons: false,
      });
      const qgcSource = sources.find((source) => (
        source.name.toLowerCase().includes("qgroundcontrol")
      ));

      if (!qgcCaptureSourcesLogged) {
        qgcCaptureSourcesLogged = true;
        console.log(
          `[QGC capture] Available windows: ${sources.map((source) => source.name).join(", ")}`,
        );
      }

      if (qgcSource && !qgcSource.thumbnail.isEmpty()) {
        if (!qgcCaptureReady) {
          qgcCaptureReady = true;
          console.log(`[QGC capture] Mirroring window: ${qgcSource.name}`);
        }
        broadcast(
          "qgc:frame",
          qgcSource.thumbnail.toJPEG(68),
        );
      }
    } catch (error) {
      console.error(`[QGC capture] ${error.message}`);
    } finally {
      qgcCaptureBusy = false;
    }
  }, 400);
}

ipcMain.handle("qgc:launch", () => launchQGroundControl());

function runMissionAdapter(action, planPath) {
  return new Promise((resolve) => {
    if (missionProcess) {
      resolve({
        ok: false,
        error: "다른 미션 작업이 진행 중입니다",
      });
      return;
    }

    const adapterPath = path.join(
      projectRoot,
      "bridge",
      "mission_adapter.py",
    );
    let bufferedOutput = "";
    let lastMessage = null;

    missionProcess = spawn(
      "python3",
      [adapterPath, action, planPath],
      {
        cwd: projectRoot,
        stdio: ["ignore", "pipe", "pipe"],
        env: process.env,
      },
    );

    missionProcess.stdout.on("data", (chunk) => {
      bufferedOutput += chunk.toString();
      const lines = bufferedOutput.split("\n");
      bufferedOutput = lines.pop();

      for (const line of lines) {
        try {
          lastMessage = JSON.parse(line);
          broadcast("mission:status", lastMessage);
        } catch {
          continue;
        }
      }
    });

    let errorOutput = "";
    missionProcess.stderr.on("data", (chunk) => {
      errorOutput += chunk.toString();
    });

    missionProcess.on("exit", (exitCode) => {
      missionProcess = null;
      resolve({
        ok: exitCode === 0,
        result: lastMessage,
        error: exitCode === 0
          ? null
          : lastMessage?.message || errorOutput.trim() || "미션 작업 실패",
      });
    });
  });
}

ipcMain.handle("mission:select", async () => {
  const result = await dialog.showOpenDialog({
    title: "QGroundControl Plan 선택",
    defaultPath: path.join(projectRoot, "plans"),
    properties: ["openFile"],
    filters: [
      { name: "QGroundControl Plan", extensions: ["plan"] },
      { name: "JSON", extensions: ["json"] },
    ],
  });

  if (result.canceled || result.filePaths.length === 0) {
    return { ok: false, canceled: true };
  }

  selectedPlanPath = result.filePaths[0];
  return runMissionAdapter("inspect", selectedPlanPath);
});

ipcMain.handle("mission:upload", () => {
  if (!selectedPlanPath) {
    return { ok: false, error: "먼저 Plan 파일을 선택하세요" };
  }
  return runMissionAdapter("upload", selectedPlanPath);
});

ipcMain.handle("mission:start", () => {
  if (!selectedPlanPath) {
    return { ok: false, error: "먼저 Plan 파일을 선택하세요" };
  }
  return runMissionAdapter("start", selectedPlanPath);
});

ipcMain.on("px4:command", (_event, command) => {
  if (rosBridge?.stdin.writable) {
    rosBridge.stdin.write(`${JSON.stringify(command)}\n`);
  }
});

app.whenReady().then(async () => {
  const baseUrl = await startServer();
  startRosBridge();
  startMavros();
  startCameraBridge();
  createWindow(baseUrl);
  setTimeout(launchQGroundControl, 1500);
  startQgcCapture();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow(baseUrl);
    }
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});

app.on("before-quit", () => {
  server?.close();
  rosBridge?.kill("SIGTERM");
  mavrosProcess?.kill("SIGTERM");
  cameraBridge?.kill("SIGTERM");
  qgcProcess?.kill("SIGTERM");
  missionProcess?.kill("SIGTERM");
  if (qgcCaptureTimer) {
    clearInterval(qgcCaptureTimer);
  }
});
