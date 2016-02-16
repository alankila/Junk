#!/usr/bin/python3.4
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import collections, gzip, os, select, socket, subprocess, sys

# buffer in memory to avoid btrfs stalls
def compress_send(cmd, out):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=open("/dev/null"))

    dq = collections.deque()

    compressor = gzip.zlib.compressobj()
    while True:
        readlist = []
        if not process.stdout.closed and len(dq) < 1000:
            readlist.append(process.stdout)
        writelist = []
        if dq:
            writelist.append(out)

        if not readlist and not writelist:
            break
        read, write, exceptional = select.select(readlist, writelist, []);

        if read:
            data = process.stdout.read(65536)
            if data:
                datacompr = compressor.compress(data)
                if datacompr:
                    dq.append(datacompr)
                #print("read[%d]: %d => %d" % (len(dq), len(data), len(datacompr)))
            else:
                dq.append(compressor.flush())

        if write:
            data = dq.popleft()
            datalen = out.send(data)
            #print("write[0]: %d / %d" % (datalen, len(data)))
            data = data[datalen:]
            if data:
                dq.appendleft(data)

    process.communicate()
    return process.returncode

def backup_client(host, port, directory):
    s = socket.create_connection((host, port), 1800)
    s.send(b"1\n")

    ack = s.recv(100).strip()
    if ack != b"1":
        print("Invalid server response to version exchange: {}".format(ack))
        return
   
    if not os.path.exists(directory + "/BACKUP"):
        process = subprocess.Popen(args=("/bin/btrfs", "subvolume", "snapshot", "-r", directory, directory + "/BACKUP"), stdout=subprocess.PIPE);
        process.communicate()
        returncode = process.returncode
        if returncode == 0:
            returncode = subprocess.call(("/bin/sync", ))
        if returncode == 0:
            returncode = compress_send(("/bin/btrfs", "send", directory + "/BACKUP"), s)
    else: 
        if os.path.exists(directory + "/BACKUP-new"):
            process = subprocess.Popen(args=("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"), stdout=subprocess.PIPE)
            process.communicate()
        process = subprocess.Popen(args=("/bin/btrfs", "subvolume", "snapshot", "-r", directory, directory + "/BACKUP-new"), stdout=subprocess.PIPE);
        process.communicate()
        returncode = process.returncode
        if returncode == 0:
            returncode = subprocess.call(("/bin/sync", ))
        if returncode == 0:
            returncode = compress_send(("/bin/btrfs", "send", "-p", directory + "/BACKUP", directory + "/BACKUP-new"), s)

    s.shutdown(socket.SHUT_WR)

    if returncode == 0:
        ack = s.recv(100).strip()
        if ack == b"OK":
            if os.path.exists(directory + "/BACKUP-new"):
                process = subprocess.Popen(args=("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP"), stdout=subprocess.PIPE)
                process.communicate()
                os.rename(directory + "/BACKUP-new", directory + "/BACKUP")
        else:
            print("Error from server: {}".format(ack))
    else:
        print("Failed at earlier step {}".format(returncode))

    if os.path.exists(directory + "/BACKUP-new"):
        subprocess.call(("/bin/btrfs", "subvolume", "delete", directory + "/BACKUP-new"))

if __name__ == "__main__":
    host, port, directory = sys.argv[1:]
    port = int(port)
    backup_client(host, port, directory)
