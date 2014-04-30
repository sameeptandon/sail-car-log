
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

def check_tracking_annolist(annolist):
    
    assert(len(annolist) > 0);

    (prev_prefix, prev_imgidx) = get_image_prefix_idx(annolist[0].imageName);

    for idx in range(1, len(annolist)):
        (cur_prefix, cur_imgidx) = get_image_prefix_idx(annolist[idx].imageName);

        if cur_imgidx <= prev_imgidx:
            if cur_prefix == prev_prefix:
                print annolist[idx - 1].imageName 
                print annolist[idx].imageName 

                print "images are not ordered by frame number\n";
                return False;

        prev_prefix, prev_imgidx = cur_prefix, cur_imgidx;
            

    return True;


