// Exercise the actual inline camera client without ROS or browser dependencies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../../../../web/index.html'), 'utf8');
const source = html.slice(html.indexOf('    class CameraStream {'), html.indexOf('    const state = {'));
const sockets = [], timers = new Map(), urls = new Set();
let sequence = 0, frames = 0;
class Socket {
  constructor(url) { this.url = url; sockets.push(this); }
  close() { if (this.onclose) this.onclose(); }
}
const context = vm.createContext({
  WebSocket: Socket, Blob,
  URL: {
    createObjectURL() { const url = 'blob:' + ++sequence; urls.add(url); return url; },
    revokeObjectURL(url) { assert.ok(urls.delete(url), 'URL must be revoked exactly once'); }
  },
  setTimeout(callback) { const id = ++sequence; timers.set(id, callback); return id; },
  clearTimeout(id) { timers.delete(id); }
});
vm.runInContext(source + '\nglobalThis.CameraStream = CameraStream;', context);
const element = {dataset: {}};
const camera = new context.CameraStream(element, 'ws://localhost:8081/camera.ws', () => frames++);
assert.equal(sockets[0].binaryType, 'blob');
sockets[0].extensions = 'permessage-deflate';
sockets[0].onopen();
assert.equal(element.dataset.wsCompression, 'permessage-deflate');
const deliver = data => sockets.at(-1).onmessage({data});
deliver(new Blob(['frame1']));
const firstUrl = element.src;
deliver(new Blob(['frame2']));
const latest = new Blob(['frame3']);
deliver(latest);
assert.equal(camera.pending, latest);
assert.equal(urls.size, 1, 'Only decode one image at a time');
element.onload();
assert.notEqual(element.src, firstUrl);
assert.equal(urls.size, 2, 'Keep displayed image until replacement loads');
element.onload();
assert.equal(frames, 2);
assert.equal(urls.size, 1);
deliver(new Blob(['invalid']));
element.onerror();
assert.equal(urls.size, 1, 'Release invalid image URL');
deliver('unexpected text');
assert.equal(camera.pending, null);
sockets[0].close();
assert.equal(timers.size, 1);
const [timerId, reconnect] = [...timers][0];
timers.delete(timerId);
reconnect();
assert.equal(sockets.length, 2);
deliver(new Blob(['after reconnect']));
element.onload();
assert.equal(frames, 3);
deliver(new Blob(['still decoding']));
camera.close();
assert.equal(urls.size, 0);
assert.equal(timers.size, 0, 'Do not reconnect after page closes');
assert.equal(element.onload, null);
console.log('Camera frontend: binary display, compression status, bounded queue, reconnect and cleanup passed.');
