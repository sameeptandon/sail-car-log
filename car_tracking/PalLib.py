import sys
import AnnoList_pb2
import AnnotationLib;

def loadPal(filename):
    _annolist = AnnoList_pb2.AnnoList();

    f = open(filename, "rb");
    _annolist.ParseFromString(f.read());
    f.close();

    return _annolist;

def savePal(filename, _annolist):
    f = open(filename, "wb");
    f.write(_annolist.SerializeToString());
    f.close();

def al2pal(annotations):
    _annolist = AnnoList_pb2.AnnoList();

    for a in annotations:
        _a = _annolist.annotation.add();
        _a.imageName = a.imageName;
		
        for r in a.rects:
            _r = _a.rect.add();
            _r.x1 = r.x1;
            _r.y1 = r.y1;
            _r.x2 = r.x2;
            _r.y2 = r.y2;
            _r.score = r.score;

            if hasattr(r, 'pal_distance'):
                _r.distance = r.pal_distance;

            if hasattr(r, 'pal_width'):
                _r.width = r.pal_width;

            if hasattr(r, 'pal_height'):
                _r.height = r.pal_height;

            if hasattr(r, 'pal_length'):
                _r.length = r.pal_length;

    return _annolist;

def pal2al(_annolist):
    annotations = [];
    
    for _a in _annolist.annotation:
        anno = AnnotationLib.Annotation()

        anno.imageName = _a.imageName;

        anno.rects = [];

        for _r in _a.rect:
            rect = AnnotationLib.AnnoRect()

            rect.x1 = _r.x1;
            rect.x2 = _r.x2;
            rect.y1 = _r.y1;
            rect.y2 = _r.y2;

            if _r.HasField("score"):
                rect.score = _r.score;

            if _r.HasField("distance"):
                rect.pal_distance = _r.distance;

            if _r.HasField("width"):
                rect.pal_width = _r.width;

            if _r.HasField("height"):
                rect.pal_height = _r.height;

            if _r.HasField("length"):
                rect.pal_length = _r.length;

            anno.rects.append(rect);

        annotations.append(anno);

    return annotations;
