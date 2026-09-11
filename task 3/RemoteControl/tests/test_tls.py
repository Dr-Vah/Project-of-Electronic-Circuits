"""Check generated cert/key, CA trust and IP SAN with a real local TLS handshake."""
from pathlib import Path
import socket, ssl, threading
root=Path(__file__).resolve().parents[1]/'main/tls'
server=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
server.load_cert_chain(root/'server-cert.pem',root/'server-key.pem')
client=ssl.create_default_context(cafile=str(root/'omni-root.pem'))
errors=[]
with socket.socket() as listener:
    listener.bind(('127.0.0.1',0));listener.listen();listener.settimeout(5)
    def serve():
        try:
            raw,_=listener.accept()
            with server.wrap_socket(raw,server_side=True) as conn:conn.sendall(b'ok')
        except Exception as e:errors.append(e)
    worker=threading.Thread(target=serve);worker.start()
    with socket.create_connection(listener.getsockname(),timeout=5) as raw:
        with client.wrap_socket(raw,server_hostname='192.168.4.1') as conn:
            assert conn.recv(2)==b'ok'
    worker.join(5);assert not worker.is_alive() and not errors,errors
assert ssl.PEM_cert_to_DER_cert((root/'omni-root.pem').read_text())==(root/'omni-root.cer').read_bytes()
print('PASS: local TLS handshake, private key match, root trust, 192.168.4.1 SAN, downloadable DER')
