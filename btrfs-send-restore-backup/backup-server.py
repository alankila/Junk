#!/usr/bin/python3.4

import datetime, gzip, os, socketserver, subprocess, sys

global directory

class BackupServer(socketserver.StreamRequestHandler):
    def handle(self):
        version = self.rfile.readline().strip()
        if version != b"1":
            self.wfile.write(b"ERR: version must be 1\n")
            return
        self.wfile.write(b"1\n")

        if os.path.exists(directory + "/BACKUP-new"):
            returncode = subprocess.call(("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

        stream = self.rfile
        process = subprocess.Popen(args=("/sbin/btrfs", "receive", directory), stdin=subprocess.PIPE)
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
        if os.path.exists(directory + "/BACKUP-new"):
            if returncode == 0:
                returncode = subprocess.call(("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP"))
            os.rename(directory + "/BACKUP-new", directory + "/BACKUP")

        if returncode == 0:
            self.wfile.write(b"OK\n");
            self.archive()
        else:
            self.wfile.write("ERR: btrfs receive returncode {}\n".format(process.returncode).encode("UTF-8"))

        # clean up if we had error and ended up with leftover backup-new
        if os.path.exists(directory + "/BACKUP-new"):
            subprocess.call(("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

    def delete_backup(self, date):
        name = directory + date.strftime(".%Y-%m-%dT%H")
        if os.path.exists(name):
            subprocess.call(("/sbin/btrfs", "subvolume", "delete", name))

    def archive(self):
        today = datetime.datetime.today();
        subprocess.call(("/sbin/btrfs", "subvolume", "snapshot", "-r", directory + "/BACKUP", directory + today.strftime(".%Y-%m-%dT%H")))

        # delete all but midnight's backups after 1 day
        date = today - datetime.timedelta(1)
        if date.hour != 0:
            self.delete_backup(date)

        # delete all but monday's backups after 1 week
        date = today.date() - datetime.timedelta(7)
        if date.weekday() != 0:
            self.delete_backup(date)

        # delete all but first week's backups after 1 month
        date = today.date() - datetime.timedelta(7*4)
        if date.day > 7:
            self.delete_backup(date)

if __name__ == "__main__":
    host, port, directory = sys.argv[1:]
    port = int(port)
    server = socketserver.TCPServer((host, port), BackupServer)
    server.serve_forever()
