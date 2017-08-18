#!/usr/bin/perl -w
#
# This tool can expose devices such as thermal line printers on the network
# to use them, write something like
#
# ./deviceproxy.pl --dev /dev/ttyACM0
#
# or
#
# ./deviceproxy.pl --dev /dev/usb/lp1
#
# And then talk to the machine's port 9100.
#

use strict;
use warnings;

use Getopt::Long;
use IO::File;
use IO::Socket::INET;
use Socket;

my $dev;
my $ip = "0.0.0.0";
my $port = 9100;
my $iolen = 8192;

my $USAGE = <<"USAGE";
$0 [ options ]

Where options is:

--dev=filename	    path to device node (required, no default)
--ip=ip-address	    the address to listen for connections (default $ip)
--port=port-number  the port to listen for connections (default $port)
--iolen=buffersize  read/write transaction size (default $iolen)

USAGE

GetOptions(
    "dev=s",  \$dev,
    "ip=s",   \$ip,
    "port=i", \$port,
    "iolen=i", \$iolen,
) || die "Failed to parse arguments\n$USAGE";
@ARGV && die "Unhandled args: @ARGV\n$USAGE";
$dev || die "--dev is required\n$USAGE";

my $socket = IO::Socket::INET->new(
    Listen    => 1,
    LocalAddr => $ip,
    LocalPort => $port,
    ReuseAddr => 1,
) || die "Failed to bind $ip:$port: $!";
$socket->blocking(0);

# only one client can connect to us, so we just use a fixed set of variables for client and buffer
my $devfh;
my $client;
my $dev_to_client = "";
my $client_to_dev = "";
while (1) {
    if (!$devfh) {
        $devfh = IO::File->new($dev, "r+");
	if ($devfh) {
	    $devfh->blocking(0);
	}
	# we failed to open the device node.
	# we are likely to try this very shortly again.
    }

    my $read = "";
    vec($read, fileno($socket), 1) = 1;
    if ($devfh) {
	vec($read, fileno($devfh), 1) = 1;
    }
    if ($client) {
	vec($read, fileno($client), 1) = 1;
    }

    my $write = "";
    if ($devfh && length $client_to_dev) {
	vec($write, fileno($devfh), 1) = 1;
    }
    if ($client && length $dev_to_client) {
	vec($write, fileno($client), 1) = 1;
    }

    # wait for something to happen for 1s
    select($read, $write, undef, 1);

    # new client.
    if (vec($read, fileno($socket), 1)) {
	$client = $socket->accept();
	$client->blocking(0);
    }
    if ($devfh && vec($read, fileno($devfh), 1)) {
	my $chunk = sysread($devfh, $dev_to_client, $iolen, length($dev_to_client));
	# EOF from device -- what should we do?
	if (!$chunk) {
	    # no data to move, sleep a little bit rather than busyloop on select
	    # this happens at least on line printer devices. It probably doesn't
	    # happen on serial devices. I don't know why Linux line printers report
	    # EOF all time time when printer has nothing to say.
            if (!length $dev_to_client && !length $client_to_dev) {
                select(undef, undef, undef, 0.1);
            }
	}
    }
    if ($client && vec($read, fileno($client), 1)) {
	my $chunk = sysread($client, $client_to_dev, $iolen, length($client_to_dev));
	# EOF/Error from client
	if (!$chunk) {
	    $client = undef;
	}
    }
    if ($devfh && vec($write, fileno($devfh), 1)) {
	my $chunk = syswrite($devfh, $client_to_dev, $iolen, 0);
        if (!$chunk) {
            $devfh = undef;
        } else {
            substr($client_to_dev, 0, $chunk) = "";
        }
    }
    if ($client && vec($write, fileno($client), 1)) {
	my $chunk = syswrite($client, $dev_to_client, $iolen, 0);
        if (!$chunk) {
            $client = undef;
        } else {
            substr($dev_to_client, 0, $chunk) = "";
        }
    }

    if (!$client && length $dev_to_client) {
	# Not buffering device data when no client is connected -- it proably doesn't matter
	# client data is important, though.
	$dev_to_client = "";
    }
}

