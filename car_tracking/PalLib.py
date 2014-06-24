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

            if hasattr(r, 'id'):
                _r.id = r.id;

            if hasattr(r, 'track_id'):
                _r.track_id = r.track_id;

            if hasattr(r, 'distance3d'):
                _r.distance3d = r.distance3d;

            if hasattr(r, 'width3d'):
                _r.width3d = r.width3d;

            if hasattr(r, 'height3d'):
                _r.height3d = r.height3d;

            if hasattr(r, 'length3d'):
                _r.length3d = r.length3d;

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

            if _r.HasField("id"):
                rect.id = _r.id;

            if _r.HasField("track_id"):
                rect.track_id = _r.track_id;

            if _r.HasField("score"):
                rect.score = _r.score;

            if _r.HasField("distance3d"):
                rect.distance3d = _r.distance3d;

            if _r.HasField("width3d"):
                rect.width3d = _r.width3d;

            if _r.HasField("height3d"):
                rect.height3d = _r.height3d;

            if _r.HasField("length3d"):
                rect.length3d = _r.length3d;

            anno.rects.append(rect);

        annotations.append(anno);

    return annotations;
