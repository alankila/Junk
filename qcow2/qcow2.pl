#!/usr/bin/perl -w

use strict;
use Carp "confess";
use Getopt::Long;

our $USAGE = <<"USAGE";
$0 <foo.qcow2>

Prints fragmentation/efficiency information of a qcow2 file.
USAGE

GetOptions() or die $USAGE;

die "Program requires exactly one non-option argument\n$USAGE" unless @ARGV == 1;

sub avg {
	if (@_ == 0) {
		return 0;
	}
	my $sum = 0;
	for (@_) {
		$sum += $_;
	}
	return $sum / @_;
}

sub read_data {
	my ($fh, $pos, $len) = @_;
	seek($fh, $pos, 0) || confess "Failed to move to position $pos: $!";
	sysread($fh, my $data, $len) == $len || confess "Failed to read $len bytes of data at $pos: $!";	
	return $data;
}

sub read_int {
	my ($fh, $pos) = @_;
	my $data = read_data($fh, $pos, 4);
	return unpack "L>", $data;
}

sub read_long {
	my ($fh, $pos) = @_;
	my $data = read_data($fh, $pos, 8);
	return unpack "Q>", $data;
}

sub read_header {
	my ($fh) = @_;
	"QFI\xfb" eq read_data($fh, 0, 4) || confess "The file is not a qcow2 file (invalid magic)";
	my $version = read_int($fh, 4);
	my $cluster_bits = read_int($fh, 20);
	my $virtual_disk_size = read_long($fh, 24);
	my $l1_size = read_int($fh, 36);
	my $l1_offset = read_long($fh, 40);
	return {
		version => $version,
		cluster_bits => $cluster_bits,
		virtual_disk_size => $virtual_disk_size,
		l1_size => $l1_size,
		l1_offset => $l1_offset,
	};
}

sub size {
	my ($size) = @_;
	if ($size >= 1024 ** 4) {
	    return sprintf("%.1f TiB", $size / (1024 ** 4));
	}
	if ($size >= 1024 ** 3) {
	    return sprintf("%.1f GiB", $size / (1024 ** 3));
	}
	if ($size >= 1024 ** 2) {
	    return sprintf("%.1f MiB", $size / (1024 ** 2));
	}
        return sprintf("%.1f KiB", $size / (1024 ** 1));
}

sub load_l1_mapping {
	my ($fh, $offset, $entries) = @_;
	my $result = [];
	for (my $pos = 0; $pos < $entries; $pos ++) {
	    my $value = read_long($fh, $offset + 8 * $pos);
	    $value &= 0x00fffffff800;
	    $result->[$pos] = $value;
	}
	return $result;
}

sub load_l2_mapping {
	my ($fh, $offset, $entries) = @_;
	
	our %cache;
	if ($cache{$offset}) {
		return $cache{$offset};
	}

	my $result = [];
	for (my $pos = 0; $pos < $entries; $pos ++) {
	    my $value = read_long($fh, $offset + 8 * $pos);
	    if ($value & 1) {
		confess "L2 cluster marked as preallocated -- shouldn't be";
	    }
	    $value &= 0x00fffffff800;
	    $result->[$pos] = $value;
	}

	return $cache{$offset} = $result;
}

sub scan_clusters {
	my ($fh, $header) = @_;

	my $cluster_bits = $header->{cluster_bits};
	my $cluster_size = 1 << $cluster_bits;
	my $virtual_disk_size = $header->{virtual_disk_size};	

	my $l1_table = load_l1_mapping($fh, $header->{l1_offset}, $header->{l1_size});

	my $last_seen_offset = 0;
	my $fragmentation_score = 0;
	my $total_mapped_clusters = 0;

	my $ordered_chunk_size = 0;
	my @ordered_chunk_size;

	for (my $offset = 0; $offset < $virtual_disk_size; $offset += $cluster_size) {
		my $l2_entries = ($cluster_size >> 3);
		my $l2_index = ($offset >> $cluster_bits) % $l2_entries;
		my $l1_index = int(($offset >> $cluster_bits) / $l2_entries);

		my $l2_table = load_l2_mapping($fh, $l1_table->[$l1_index], $l2_entries);
		my $cluster_offset = $l2_table->[$l2_index];

		# unmapped clusters don't count to frag score
		if ($cluster_offset == 0) {
			next;
		}
		
		$total_mapped_clusters += 1;
		
		if ($cluster_offset < $last_seen_offset || $cluster_offset > $last_seen_offset + $cluster_size) {
			$fragmentation_score ++;
			push @ordered_chunk_size, $ordered_chunk_size;
			$ordered_chunk_size = 0;
		}
		$last_seen_offset = $cluster_offset;
		$ordered_chunk_size += $cluster_size;
	}
	push @ordered_chunk_size, $ordered_chunk_size;

	return $fragmentation_score / $total_mapped_clusters, avg(@ordered_chunk_size);
}

sub main {
	my ($file) = @_;
	open(my ($fh), "<", $file) || confess "Could not open qcow2 file '$file': $!";
	my $header = read_header($fh);

	print sprintf("Version: %d\n", $header->{version});
	print sprintf("Cluster size: %s\n", size(1 << $header->{cluster_bits}));
	print sprintf("Virtual disk size: %s\n", size($header->{virtual_disk_size}));
	print "\n";
	print "Analyzing...\n";

	my ($fragmentation_score, $avg_contiguous) = scan_clusters($fh, $header);

	print "\n";
	print sprintf("Virtual<->Physical disordering metric: %.2f %%\n", $fragmentation_score * 100);
	print sprintf("Average contiguous cluster size: %s\n", size($avg_contiguous));
}

main($ARGV[0]);

