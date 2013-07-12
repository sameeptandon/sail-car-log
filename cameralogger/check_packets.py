import os
import sys

packet_prefix = '#MARK1PVAA'

def main():
    fname = open(sys.argv[1], 'rb')
    packets = fname.readlines()
    fname.close()

    valid_count = 0
    invalid_count = 0
    invalid_line_nums = []
    for i in xrange(len(packets)):
        packet = packets[i]
        if packet.startswith('INVALID'):
            index = packet.find(packet_prefix)
            valid = False
            if index != -1:
                post_header_index = packet.find(';', index)
                post_header = packet[post_header_index+1:]
                parts = post_header.split(',')

                if len(parts) == 12:
                    valid = True
            if valid:
                valid_count += 1
            else:
                invalid_line_nums.append(i)
                invalid_count += 1
    
    print 'Potentially valid packets marked invalid: ' + str(valid_count)
    print 'Almost definitely invalid packets: ' + str(invalid_count)
    print 'at lines ' + str(invalid_line_nums)

if __name__ == '__main__':
    main()
