/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class vectornav_lcmt implements lcm.lcm.LCMEncodable
{
    public float q[];
    public float w[];
    public float a[];
 
    public vectornav_lcmt()
    {
        q = new float[4];
        w = new float[3];
        a = new float[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf57906decbf7ebdcL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.vectornav_lcmt.class))
            return 0L;
 
        classes.add(lcmtypes.vectornav_lcmt.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.q[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.w[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeFloat(this.a[a]); 
        }
 
    }
 
    public vectornav_lcmt(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public vectornav_lcmt(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.vectornav_lcmt _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.vectornav_lcmt o = new lcmtypes.vectornav_lcmt();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.q = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.q[a] = ins.readFloat();
        }
 
        this.w = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.w[a] = ins.readFloat();
        }
 
        this.a = new float[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.a[a] = ins.readFloat();
        }
 
    }
 
    public lcmtypes.vectornav_lcmt copy()
    {
        lcmtypes.vectornav_lcmt outobj = new lcmtypes.vectornav_lcmt();
        outobj.q = new float[(int) 4];
        System.arraycopy(this.q, 0, outobj.q, 0, 4); 
        outobj.w = new float[(int) 3];
        System.arraycopy(this.w, 0, outobj.w, 0, 3); 
        outobj.a = new float[(int) 3];
        System.arraycopy(this.a, 0, outobj.a, 0, 3); 
        return outobj;
    }
 
}

