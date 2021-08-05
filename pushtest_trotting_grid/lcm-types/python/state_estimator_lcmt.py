"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class state_estimator_lcmt(object):
    __slots__ = ["p", "vWorld", "vBody", "rpy", "omegaBody", "omegaWorld", "quat"]

    def __init__(self):
        self.p = [ 0.0 for dim0 in range(3) ]
        self.vWorld = [ 0.0 for dim0 in range(3) ]
        self.vBody = [ 0.0 for dim0 in range(3) ]
        self.rpy = [ 0.0 for dim0 in range(3) ]
        self.omegaBody = [ 0.0 for dim0 in range(3) ]
        self.omegaWorld = [ 0.0 for dim0 in range(3) ]
        self.quat = [ 0.0 for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(state_estimator_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3f', *self.p[:3]))
        buf.write(struct.pack('>3f', *self.vWorld[:3]))
        buf.write(struct.pack('>3f', *self.vBody[:3]))
        buf.write(struct.pack('>3f', *self.rpy[:3]))
        buf.write(struct.pack('>3f', *self.omegaBody[:3]))
        buf.write(struct.pack('>3f', *self.omegaWorld[:3]))
        buf.write(struct.pack('>4f', *self.quat[:4]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != state_estimator_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return state_estimator_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = state_estimator_lcmt()
        self.p = struct.unpack('>3f', buf.read(12))
        self.vWorld = struct.unpack('>3f', buf.read(12))
        self.vBody = struct.unpack('>3f', buf.read(12))
        self.rpy = struct.unpack('>3f', buf.read(12))
        self.omegaBody = struct.unpack('>3f', buf.read(12))
        self.omegaWorld = struct.unpack('>3f', buf.read(12))
        self.quat = struct.unpack('>4f', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if state_estimator_lcmt in parents: return 0
        tmphash = (0xc329843a643aae5b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if state_estimator_lcmt._packed_fingerprint is None:
            state_estimator_lcmt._packed_fingerprint = struct.pack(">Q", state_estimator_lcmt._get_hash_recursive([]))
        return state_estimator_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

