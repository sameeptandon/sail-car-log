def inc_image_name(s, n):
    pos1 = s.rfind('_');
    pos2 = s.rfind('.');

    num = int(s[pos1+1:pos2]) + n
    numlen = pos2 - pos1 - 1;
    res = s[:pos1+1] + str(num).zfill(numlen) + s[pos2:];
    return res;

def get_image_prefix_idx(s):
    pos1 = s.rfind('_');
    pos2 = s.rfind('.');
    assert(pos1 != -1 and pos2 != -1);

    return (s[:pos1], int(s[pos1+1:pos2]));
