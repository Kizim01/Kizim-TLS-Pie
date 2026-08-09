#!/usr/bin/env python3
"""
Minimal pcap reader: walks a tcpdump capture and yields UDP payloads.

Stdlib only, on purpose. The Pi needs this to build the phone preview and the
Pi runs offline on a phone hotspot, so a pip dependency there is a liability
rather than a convenience. The workstation side is free to use
`velodyne-decoder` (BSD-3, aarch64 wheels, handles dual returns and per-point
timing properly) for the full-resolution cloud -- that is the right tool for
113 million points, and the wrong tool for a Pi 4 that only wants 150,000.

WHY IT WALKS THE WHOLE FILE
---------------------------
Records are variable length, so there is no way to seek to the Nth packet
without reading the ones before it. A 6.5 minute scan is ~360 MB and ~290,000
records; walking it takes a handful of seconds on a Pi and there is no cleverer
option that stays correct. Decimation saves the DECODE cost, which is the part
that actually hurts, not the read.

WHAT IT HANDLES
---------------
Both byte orders and both timestamp resolutions (libpcap grew nanosecond
variants), Ethernet and Linux cooked captures, and VLAN tags. It does NOT
handle pcapng -- tcpdump writes classic pcap unless asked otherwise, and this
raises a clear error rather than guessing if it ever meets one.

Truncated files are read up to the truncation and no further. That is the
normal outcome of cutting S1 mid-scan, so it must not be an exception: the
packets before the cut are perfectly good data.
"""

import struct


# The magic is the VALUE 0xA1B2C3D4 stored in the writing host's native byte
# order, so the byte order is discovered by reading it one fixed way and seeing
# which value comes back. These four are what a little-endian read produces:
#
#   file bytes D4 C3 B2 A1  ->  0xA1B2C3D4  ->  little-endian file
#   file bytes A1 B2 C3 D4  ->  0xD4C3B2A1  ->  big-endian file
#
# Re-reading the header big-endian to test the big-endian case swaps twice and
# never matches -- which is exactly the bug the byte-order test caught.
PCAP_MAGIC_LE = 0xA1B2C3D4          # microsecond, little endian
PCAP_MAGIC_BE = 0xD4C3B2A1          # microsecond, big endian
PCAP_MAGIC_NS_LE = 0xA1B23C4D       # nanosecond, little endian
PCAP_MAGIC_NS_BE = 0x4D3CB2A1       # nanosecond, big endian
PCAPNG_MAGIC = 0x0A0D0D0A

LINKTYPE_ETHERNET = 1
LINKTYPE_LINUX_SLL = 113
LINKTYPE_LINUX_SLL2 = 276

ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_VLAN = 0x8100
ETHERTYPE_QINQ = 0x88A8
IP_PROTO_UDP = 17

READ_CHUNK = 1 << 20


class PcapError(ValueError):
    pass


def _parse_ipv4_udp(data, offset):
    """(src_port, dst_port, payload) from an IPv4 frame, or None."""
    if len(data) < offset + 20:
        return None
    version_ihl = data[offset]
    if (version_ihl >> 4) != 4:
        return None                      # IPv6; the VLP-16 does not use it
    ihl = (version_ihl & 0x0F) * 4
    if ihl < 20 or len(data) < offset + ihl + 8:
        return None
    if data[offset + 9] != IP_PROTO_UDP:
        return None

    # Fragmented datagrams are dropped rather than reassembled. A VLP-16 packet
    # is 1206 bytes and never fragments on a normal 1500 byte MTU, so meeting
    # one means something is wrong upstream and silently half-decoding it would
    # be worse than skipping it.
    flags_frag = struct.unpack_from(">H", data, offset + 6)[0]
    if (flags_frag & 0x1FFF) != 0:
        return None

    udp = offset + ihl
    src_port, dst_port, udp_len = struct.unpack_from(">HHH", data, udp)
    payload = data[udp + 8:]
    # udp_len covers the 8 byte header. Trust it when it is sane, since a
    # snaplen-truncated capture can leave the buffer shorter than it claims.
    body = udp_len - 8
    if 0 <= body <= len(payload):
        payload = payload[:body]
    return (src_port, dst_port, payload)


def _parse_link(data, linktype):
    if linktype == LINKTYPE_ETHERNET:
        if len(data) < 14:
            return None
        ethertype = struct.unpack_from(">H", data, 12)[0]
        offset = 14
        # Up to two stacked VLAN tags, which is as deep as anything sane goes.
        for _ in range(2):
            if ethertype in (ETHERTYPE_VLAN, ETHERTYPE_QINQ):
                if len(data) < offset + 4:
                    return None
                ethertype = struct.unpack_from(">H", data, offset + 2)[0]
                offset += 4
            else:
                break
        if ethertype != ETHERTYPE_IPV4:
            return None
        return _parse_ipv4_udp(data, offset)

    if linktype == LINKTYPE_LINUX_SLL:
        if len(data) < 16:
            return None
        if struct.unpack_from(">H", data, 14)[0] != ETHERTYPE_IPV4:
            return None
        return _parse_ipv4_udp(data, 16)

    if linktype == LINKTYPE_LINUX_SLL2:
        if len(data) < 20:
            return None
        if struct.unpack_from(">H", data, 0)[0] != ETHERTYPE_IPV4:
            return None
        return _parse_ipv4_udp(data, 20)

    raise PcapError("unsupported link type %d" % linktype)


def read_header(handle):
    """(endian_prefix, nanoseconds, snaplen, linktype) from a pcap file."""
    header = handle.read(24)
    if len(header) < 24:
        raise PcapError("file is too short to be a pcap")

    magic = struct.unpack("<I", header[:4])[0]
    if magic == PCAPNG_MAGIC:
        raise PcapError(
            "this is a pcapng file, not classic pcap. Re-capture with "
            "`tcpdump -w` (which writes classic pcap), or convert it with "
            "`editcap -F pcap in.pcapng out.pcap`.")

    if magic == PCAP_MAGIC_LE:
        endian, nanos = "<", False
    elif magic == PCAP_MAGIC_NS_LE:
        endian, nanos = "<", True
    elif magic == PCAP_MAGIC_BE:
        endian, nanos = ">", False
    elif magic == PCAP_MAGIC_NS_BE:
        endian, nanos = ">", True
    else:
        raise PcapError("not a pcap file (bad magic 0x%08X)" % magic)

    _, _, _, _, snaplen, linktype = struct.unpack(endian + "HHiIII",
                                                  header[4:24])
    return endian, nanos, snaplen, linktype


def udp_packets(path, port=None, stride=1, want_index=None):
    """
    Yield (index, epoch_seconds, payload) for UDP packets in `path`.

    `port`       keep only this destination port (None keeps all)
    `stride`     decode one packet in `stride` of those that match
    `want_index` optional callable(index) -> bool, overriding `stride`

    `index` counts matching packets, not file records, so it is stable under a
    change of capture filter. The record header is always parsed -- that is how
    you find the next record -- but the payload is only sliced for packets that
    are actually wanted.
    """
    stride = max(1, int(stride))
    if want_index is None:
        def want_index(i):
            return (i % stride) == 0

    with open(path, "rb") as handle:
        endian, nanos, _, linktype = read_header(handle)
        record = struct.Struct(endian + "IIII")
        scale = 1e-9 if nanos else 1e-6
        index = 0

        while True:
            head = handle.read(16)
            if len(head) < 16:
                return                    # clean EOF, or a truncated record
            ts_sec, ts_frac, incl_len, _ = record.unpack(head)
            if incl_len == 0:
                continue
            body = handle.read(incl_len)
            if len(body) < incl_len:
                return                    # truncated mid-record: stop here

            parsed = _parse_link(body, linktype)
            if parsed is None:
                continue
            _, dst_port, payload = parsed
            if port is not None and dst_port != port:
                continue

            i = index
            index += 1
            if not want_index(i):
                continue
            yield (i, ts_sec + ts_frac * scale, payload)


def count_udp_packets(path, port=None):
    """
    How many matching packets the file holds. Walks the whole file.

    Only worth calling for `--stats` and tests. Choosing a decimation stride
    does NOT use this: knowing the count exactly would cost a second pass over
    360 MB to save nothing, since voxel downsampling lands the point budget
    afterwards regardless. Use `estimate_packet_count` for that.
    """
    total = 0
    for _ in udp_packets(path, port=port, want_index=lambda i: True):
        total += 1
    return total


def estimate_packet_count(path, bytes_per_packet=1264):
    """
    Rough packet count from file size, without reading the file.

    A VLP-16 data packet is 1206 bytes plus 42 of Ethernet/IP/UDP plus 16 of
    pcap record header. Position packets and any other traffic on the wire make
    this an over-estimate of the DATA packets, which is the safe direction:
    it biases the stride towards decoding fewer points than the budget allows,
    never more.
    """
    import os
    try:
        size = os.path.getsize(path)
    except OSError:
        return 0
    return max(0, (size - 24) // bytes_per_packet)
