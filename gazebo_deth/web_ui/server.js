#!/usr/bin/env node
// Event/demo server: serves web_ui/ as static files and proxies
// /rosbridge websocket upgrades to the local rosbridge server (default
// 127.0.0.1:9090). Exists so a single "ssh -R 80:localhost:PORT ..." tunnel
// (or ngrok/localhost.run/etc.) exposes both the page and the ROS websocket
// under one origin -- see js/ros.js, which points at /rosbridge whenever the
// page isn't loaded from localhost.
//
// Usage: node server.js [port]
// Env vars: PROXY_PORT, ROSBRIDGE_HOST, ROSBRIDGE_PORT

'use strict';
const http = require('http');
const net = require('net');
const fs = require('fs');
const path = require('path');

const WEB_ROOT = __dirname;
const PROXY_PORT = Number(process.argv[2]) || Number(process.env.PROXY_PORT) || 8080;
const ROSBRIDGE_HOST = process.env.ROSBRIDGE_HOST || '127.0.0.1';
const ROSBRIDGE_PORT = Number(process.env.ROSBRIDGE_PORT) || 9090;
const ROSBRIDGE_PATH = '/rosbridge';

const MIME = {
    '.html': 'text/html',
    '.js': 'application/javascript',
    '.css': 'text/css',
    '.json': 'application/json',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
    '.glb': 'model/gltf-binary',
    '.gltf': 'model/gltf+json',
};

const server = http.createServer((req, res) => {
    let reqPath = decodeURIComponent(req.url.split('?')[0]);
    if (reqPath === '/') reqPath = '/index.html';
    const filePath = path.normalize(path.join(WEB_ROOT, reqPath));
    if (!filePath.startsWith(WEB_ROOT)) {
        res.writeHead(403);
        res.end('Forbidden');
        return;
    }
    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(404);
            res.end('Not found');
            return;
        }
        res.writeHead(200, { 'Content-Type': MIME[path.extname(filePath)] || 'application/octet-stream' });
        res.end(data);
    });
});

// Raw TCP passthrough for the websocket upgrade: forward the handshake
// (rewriting the request line to "/" since rosbridge doesn't route on path)
// then pipe both sockets together.
server.on('upgrade', (req, clientSocket, head) => {
    if (!req.url.startsWith(ROSBRIDGE_PATH)) {
        clientSocket.destroy();
        return;
    }

    const backend = net.connect(ROSBRIDGE_PORT, ROSBRIDGE_HOST, () => {
        const headerLines = ['GET / HTTP/1.1'];
        for (let i = 0; i < req.rawHeaders.length; i += 2) {
            if (/^host$/i.test(req.rawHeaders[i])) continue;
            headerLines.push(`${req.rawHeaders[i]}: ${req.rawHeaders[i + 1]}`);
        }
        headerLines.push(`Host: ${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`, '', '');
        backend.write(headerLines.join('\r\n'));
        if (head && head.length) backend.write(head);
        clientSocket.pipe(backend);
        backend.pipe(clientSocket);
    });

    backend.on('error', () => clientSocket.destroy());
    clientSocket.on('error', () => backend.destroy());
});

server.listen(PROXY_PORT, () => {
    console.log(`web_ui event server on http://0.0.0.0:${PROXY_PORT}`);
    console.log(`  static files: ${WEB_ROOT}`);
    console.log(`  ${ROSBRIDGE_PATH} -> ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`);
});
