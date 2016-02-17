#!/usr/bin/python3

import datetime, gzip, os, socketserver, socket, subprocess, sys

global directory

class BackupServer(socketserver.StreamRequestHandler):
    def handle(self):
        # Turn on keepalives so that we eventually recover on client crashing or network loss
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        version = self.rfile.readline().strip()
        if version != b"1":
            self.wfile.write(b"ERR: version must be 1\n")
            return
        self.wfile.write(b"1\n")

        if os.path.exists(directory + "/BACKUP-new"):
            returncode = subprocess.call(("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

        stream = self.rfile
        process = subprocess.Popen(args=("/bin/btrfs", "receive", directory), stdin=subprocess.PIPE)
        decompressor = gzip.zlib.decompressobj()
        while True:
            data = stream.read(65536)
            if not data:
                break
            process.stdin.write(decompressor.decompress(data))
        process.stdin.write(decompressor.flush())
        process.stdin.close()
        process.communicate()
        returncode = process.returncode
        
        # when server is used in incremental mode, the new file is called BACKUP-new
        # if it's the initial sync, there is no file like this
        if returncode == 0:
            if os.path.exists(directory + "/BACKUP-new"):
                if os.path.exists(directory + "/BACKUP"):
                    returncode = subprocess.call(("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP"))
                os.rename(directory + "/BACKUP-new", directory + "/BACKUP")

        if returncode == 0:
            self.wfile.write(b"OK\n")
            self.wfile.close()
            self.archive()
        else:
            self.wfile.write("ERR: btrfs receive returncode {}\n".format(process.returncode).encode("UTF-8"))

        # clean up if we had error and ended up with leftover backup-new
        if os.path.exists(directory + "/BACKUP-new"):
            subprocess.call(("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

    def delete_backup(self, date):
        name = directory + date.strftime(".%Y-%m-%dT%H")
        if os.path.exists(name):
            subprocess.call(("/bin/btrfs", "subvolume", "delete", name))

    def archive(self):
        today = datetime.datetime.today();
        subprocess.call(("/bin/btrfs", "subvolume", "snapshot", "-r", directory + "/BACKUP", directory + today.strftime(".%Y-%m-%dT%H")))

        # delete all but midnight's backups after 1 day
        date = today - datetime.timedelta(1)
        if date.hour != 0:
            self.delete_backup(date)

        # delete all but first week's backups after 1 month
        date = today.date() - datetime.timedelta(7*4)
        if date.weekday() != 0 or date.day > 7:
            self.delete_backup(date)

        # unconditionally delete after 365 days
        date = today.date() - datetime.timedelta(365)
        self.delete_backup(date)

if __name__ == "__main__":
    host, port, directory = sys.argv[1:]
    port = int(port)
    server = socketserver.TCPServer((host, port), BackupServer)
    server.allow_reuse_address = True
    server.serve_forever()
