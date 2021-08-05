"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class heightmap_t(object):
    __slots__ = ["map", "robot_loc"]

    def __init__(self):
        self.map = [ [ 0.0 for dim1 in range(100) ] for dim0 in range(100) ]
        self.robot_loc = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(heightmap_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        for i0 in range(100):
            buf.write(struct.pack('>100d', *self.map[i0][:100]))
        buf.write(struct.pack('>3d', *self.robot_loc[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != heightmap_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return heightmap_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = heightmap_t()
        self.map = []
        for i0 in range(100):
            self.map.append(struct.unpack('>100d', buf.read(800)))
        self.robot_loc = struct.unpack('>3d', buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if heightmap_t in parents: return 0
        tmphash = (0x9dc86e2cda9acee7) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if heightmap_t._packed_fingerprint is None:
            heightmap_t._packed_fingerprint = struct.pack(">Q", heightmap_t._get_hash_recursive([]))
        return heightmap_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
