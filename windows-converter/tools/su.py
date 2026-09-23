# Minimal client for the sketchup-mcp2 extension: 4-byte big-endian length + JSON-RPC 2.0, hello first.
import json, socket, struct, sys
PORT = 9877
def rpc(sock, method, params, i):
    body = json.dumps({"jsonrpc": "2.0", "method": method, "params": params, "id": i}).encode()
    sock.sendall(struct.pack(">I", len(body)) + body)
    head = b""
    while len(head) < 4:
        c = sock.recv(4 - len(head)); assert c, "closed"; head += c
    n = struct.unpack(">I", head)[0]; data = b""
    while len(data) < n:
        c = sock.recv(min(1 << 20, n - len(data))); assert c, "closed"; data += c
    return json.loads(data)
def session(calls, timeout=60):
    s = socket.create_connection(("127.0.0.1", PORT), timeout=timeout)
    out = [rpc(s, "hello", {"client_version": sys.argv[1] if len(sys.argv) > 1 and sys.argv[1][0].isdigit() else "0.3.1"}, 0)]
    for k, (m, p) in enumerate(calls, 1):
        out.append(rpc(s, m, p, k))
    s.close(); return out
if __name__ == "__main__":
    calls = json.loads(sys.stdin.read() or "[]")
    for r in session(calls):
        print(json.dumps(r)[:4000])
