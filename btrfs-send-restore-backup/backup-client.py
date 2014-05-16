#!/usr/bin/python3.3

import gzip, os, socket, subprocess, sys

# buffer in memory to avoid btrfs stalls
def compress_send(cmd, out):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=open("/dev/null"))

    frags = []
    compressor = gzip.zlib.compressobj()
    while True:
        data = process.stdout.read(65536)
        if not data:
           break
        frags.append(compressor.compress(data))
    frags.append(compressor.flush())

    # when ready, write to output
    for data in frags:
        out.sendall(data)

    process.communicate()
    return process.returncode

def backup_client(host, port, directory):
    s = socket.create_connection((host, port), 600)
    s.send(b"1\n")

    ack = s.recv(100).strip()
    if ack != b"1":
        print("Invalid server response to version exchange: {}".format(ack))
        return
   
    if not os.path.exists(directory + "/BACKUP"):
        process = subprocess.Popen(args=("/sbin/btrfs", "subvolume", "snapshot", "-r", directory, directory + "/BACKUP"), stdout=subprocess.PIPE);
        process.communicate()
        returncode = process.returncode
        if returncode == 0:
            returncode = subprocess.call(("/bin/sync", ))
        if returncode == 0:
            returncode = compress_send(("/sbin/btrfs", "send", directory + "/BACKUP"), s)
    else: 
        if os.path.exists(directory + "/BACKUP-new"):
            process = subprocess.Popen(args=("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"), stdout=subprocess.PIPE)
            process.communicate()
        process = subprocess.Popen(args=("/sbin/btrfs", "subvolume", "snapshot", "-r", directory, directory + "/BACKUP-new"), stdout=subprocess.PIPE);
        process.communicate()
        returncode = process.returncode
        if returncode == 0:
            returncode = subprocess.call(("/bin/sync", ))
        if returncode == 0:
            returncode = compress_send(("/sbin/btrfs", "send", "-p", directory + "/BACKUP", directory + "/BACKUP-new"), s)

    s.shutdown(socket.SHUT_WR)

    if returncode == 0:
        ack = s.recv(100).strip()
        if ack == b"OK":
            if os.path.exists(directory + "/BACKUP-new"):
                process = subprocess.Popen(args=("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP"), stdout=subprocess.PIPE)
                process.communicate()
                os.rename(directory + "/BACKUP-new", directory + "/BACKUP")
        else:
            print("Error from server: {}".format(ack))
    else:
        print("Failed at earlier step {}".format(returncode))

    if os.path.exists(directory + "/BACKUP-new"):
        subprocess.call(("/sbin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

if __name__ == "__main__":
    host, port, directory = sys.argv[1:]
    port = int(port)
    backup_client(host, port, directory)
