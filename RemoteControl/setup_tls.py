"""Generate a project-local CA and ESP32 certificate; never install trust on the PC."""
from pathlib import Path
import shutil, subprocess, argparse

ROOT=Path(__file__).resolve().parent
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--openssl');args=ap.parse_args()
    exe=args.openssl or shutil.which('openssl')
    if not exe: raise SystemExit('OpenSSL missing. Run: python setup_tls.py --openssl PATH_TO_OPENSSL')
    private=ROOT/'.tls';out=ROOT/'main/tls'
    private.mkdir(exist_ok=True);out.mkdir(exist_ok=True)
    def run(*a):subprocess.run([exe,*map(str,a)],check=True,stdout=subprocess.DEVNULL)
    ca=out/'omni-root.pem';cakey=private/'root-key.pem'
    conf=private/'openssl.cnf';conf.write_text('[req]\ndistinguished_name=dn\n[dn]\n',encoding='ascii')
    if ca.exists() and not cakey.exists():raise SystemExit('Restore .tls/root-key.pem; existing CA will not be replaced.')
    if not ca.exists():
        if not cakey.exists():run('ecparam','-name','prime256v1','-genkey','-noout','-out',cakey)
        run('req','-config',conf,'-new','-x509','-sha256','-days','3650','-key',cakey,'-out',ca,
            '-subj','/CN=Omni Remote Local CA','-addext','basicConstraints=critical,CA:TRUE,pathlen:0',
            '-addext','keyUsage=critical,keyCertSign,cRLSign')
    key=out/'server-key.pem';cert=out/'server-cert.pem'
    if not key.exists():run('ecparam','-name','prime256v1','-genkey','-noout','-out',key)
    # Reuse valid leaf certificate so ordinary builds do not change phone trust.
    if not cert.exists() or subprocess.run([exe,'x509','-checkend','604800','-noout','-in',str(cert)],stdout=subprocess.DEVNULL).returncode:
        csr=private/'server.csr';ext=private/'server.ext'
        ext.write_text('basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature\nextendedKeyUsage=serverAuth\nsubjectAltName=IP:192.168.4.1,DNS:omni-remote.local\n',encoding='ascii')
        run('req','-config',conf,'-new','-key',key,'-out',csr,'-subj','/CN=192.168.4.1')
        run('x509','-req','-in',csr,'-CA',ca,'-CAkey',cakey,'-CAserial',private/'serial',
            '-CAcreateserial','-out',cert,'-days','365','-sha256','-extfile',ext)
    run('x509','-in',ca,'-outform','DER','-out',out/'omni-root.cer')
    run('verify','-CAfile',ca,'-verify_ip','192.168.4.1',cert)
    print('TLS ready. Install only main/tls/omni-root.cer on your phone. Keep private keys local.')
if __name__=='__main__':main()
