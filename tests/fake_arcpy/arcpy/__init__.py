"""Minimal functional arcpy stand-in for testing the .pyt outside ArcGIS Pro."""
import os, tempfile, numpy as np
from pyproj import Transformer, CRS
class ExecuteError(Exception): pass
MSGS=[]
def AddMessage(s): MSGS.append(("I",s))
def AddWarning(s): MSGS.append(("W",s))
class _Filter:
    def __init__(s): s.type=None; s.list=[]
class Parameter:
    def __init__(s, name, displayName, datatype, parameterType, direction, multiValue=False, category=None):
        s.name=name; s.displayName=displayName; s.datatype=datatype; s.parameterType=parameterType
        s.direction=direction; s.multiValue=multiValue; s.category=category; s.filter=_Filter()
        s.value=None; s.enabled=True; s.altered=False; s.hasBeenValidated=False; s.msg=None
        s.columns=[]  # self_columns
    @property
    def valueAsText(s):
        if s.value is None: return None
        if s.multiValue and isinstance(s.value,(list,tuple)): return ";".join(map(str,s.value))
        return str(s.value)
    @property
    def values(s):
        if s.datatype == "GPValueTable":
            return [list(r) for r in s.value] if s.value is not None else None
        return list(s.value) if isinstance(s.value,(list,tuple)) else ([s.value] if s.value is not None else None)
    def setWarningMessage(s,m): s.msg=("W",m)
    def setErrorMessage(s,m): s.msg=("E",m)
class SpatialReference:
    def __init__(s, code=4326):
        s.factoryCode=code; s.crs=CRS.from_epsg(code); s.name=s.crs.name
        s.type="Geographic" if s.crs.is_geographic else "Projected"
    def __eq__(s,o): return isinstance(o,SpatialReference) and o.factoryCode==s.factoryCode
class Point:
    def __init__(s,X=0.0,Y=0.0): s.X=X; s.Y=Y
class Array(list):
    def add(s,x): s.append(x)
class _Extent:
    def __init__(s,xs,ys): s.XMin,s.XMax,s.YMin,s.YMax=min(xs),max(xs),min(ys),max(ys)
def _tf(a,b): return Transformer.from_crs(a.crs,b.crs,always_xy=True)
class _Geom:
    def _pts(s): return [p for part in s.parts for p in part if p is not None]
    @property
    def extent(s): pts=s._pts(); return _Extent([p.X for p in pts],[p.Y for p in pts])
    @property
    def firstPoint(s): return s._pts()[0]
    @property
    def pointCount(s): return len(s._pts())
    @property
    def isMultipart(s): return len(s.parts)>1
    def __iter__(s): return iter(s.parts)
    def projectAs(s, sr, tr=""):
        t=_tf(s.spatialReference,sr)
        parts=[[None if p is None else Point(*t.transform(p.X,p.Y)) for p in part] for part in s.parts]
        g=object.__new__(type(s)); g.parts=parts; g.spatialReference=sr; return g
    @property
    def centroid(s): pts=s._pts(); return Point(sum(p.X for p in pts)/len(pts), sum(p.Y for p in pts)/len(pts))
    def union(s,o):
        g=object.__new__(type(s)); g.parts=s.parts+o.parts; g.spatialReference=s.spatialReference; return g
    @property
    def WKT(s): return ""
class PointGeometry(_Geom):
    def __init__(s,pt,sr=None): s.parts=[[pt]]; s.spatialReference=sr
class Polyline(_Geom):
    def __init__(s,arr,sr=None):
        s.parts=[list(arr)] if arr and isinstance(arr[0],Point) else [list(a) for a in arr]; s.spatialReference=sr
class Polygon(_Geom):
    def __init__(s,arr,sr=None):
        if arr and isinstance(arr[0],Point): s.parts=[list(arr)]
        else:   # array of rings -> one part, rings separated by None
            part=[]
            for i,r in enumerate(arr):
                if i: part.append(None)
                part+=list(r)
            s.parts=[part]
        s.spatialReference=sr
class _Env:
    workspace=None; overwriteOutput=False
    scratchFolder=tempfile.mkdtemp(); scratchGDB=os.path.join(scratchFolder,"scratch.gdb")
env=_Env()
FCS={}   # path -> dict(shape, sr, fields, rows)
INPUTS={}  # name -> dict(shapeType, sr, geoms)
class _Desc:
    def __init__(s,d): s.spatialReference=d["sr"]; s.shapeType=d["shapeType"]
def Describe(x): return _Desc(INPUTS[x])
def Exists(p): return p in FCS
def ValidateTableName(n,ws=None): return n.replace(" ","_")
def ListTransformations(a,b,ext=None): return []
class _MG:
    @staticmethod
    def CreateFeatureclass(gdb,name,shape,spatial_reference=None):
        FCS[os.path.join(gdb,name)]={"shape":shape,"sr":spatial_reference,"fields":["SHAPE"],"rows":[]}
    @staticmethod
    def CreateTable(gdb,name):
        FCS[os.path.join(gdb,name)]={"shape":None,"sr":None,"fields":[],"rows":[]}
    @staticmethod
    def AddField(path,f,t): FCS[path]["fields"].append(f)
    @staticmethod
    def Delete(p): FCS.pop(p,None)
management=_MG()
class _Cur:
    def __init__(s,fc,fields): s.fc=fc; s.fields=fields
    def __enter__(s): return s
    def __exit__(s,*a): pass
class _SC(_Cur):
    def __iter__(s):
        d = INPUTS[s.fc]; attrs = d.get("attrs") or [{} for _ in d["geoms"]]
        return iter([tuple([g] + [a.get(f) for f in s.fields[1:]]) for g, a in zip(d["geoms"], attrs)])
class _IC(_Cur):
    def insertRow(s,row):
        f=FCS[s.fc]; assert len(row)==len(s.fields), (row,s.fields)
        if f["shape"] is None:
            for name in s.fields: assert name in f["fields"], name
            f["rows"].append(row); return
        for name in s.fields[1:]: assert name in f["fields"], name
        g=row[0]
        if s.fields[0]=="SHAPE@XY": assert len(g)==2 and all(isinstance(v,float) for v in g)
        else: assert isinstance(g,_Geom) and g.pointCount>0
        f["rows"].append(row)
class da:
    SearchCursor=_SC; InsertCursor=_IC
class Raster:
    def __init__(s,path):
        from osgeo import gdal, osr; ds=gdal.Open(path); gt=ds.GetGeoTransform()
        s.arr=ds.ReadAsArray().astype(float); s.gt=gt
        s.meanCellWidth=gt[1]; s.meanCellHeight=-gt[5]
        sr=osr.SpatialReference(wkt=ds.GetProjection()); sr.AutoIdentifyEPSG()
        s.spatialReference=SpatialReference(int(sr.GetAuthorityCode(None) or 4326))
        s.extent=_Extent([gt[0],gt[0]+gt[1]*s.arr.shape[1]],[gt[3]+gt[5]*s.arr.shape[0],gt[3]])
def RasterToNumPyArray(r, ll, ncols, nrows, nodata_to_value=None):
    c0=int(round((ll.X-r.gt[0])/r.gt[1])); top=ll.Y+nrows*r.meanCellHeight
    r0=int(round((r.gt[3]-top)/r.meanCellHeight))
    out=np.full((nrows,ncols),nodata_to_value,float)
    sub=r.arr[max(r0,0):r0+nrows, max(c0,0):c0+ncols]; out[:sub.shape[0],:sub.shape[1]]=sub; return out
class mp:
    class ArcGISProject:
        def __init__(s,x): raise RuntimeError("no project")

class _NPRaster:
    def __init__(s, arr, ll, cw, ch): s.arr, s.ll, s.cw, s.ch = arr, ll, cw, ch
    def save(s, path):
        from osgeo import gdal
        ds = gdal.GetDriverByName("GTiff").Create(path, s.arr.shape[1], s.arr.shape[0], 1, gdal.GDT_Float32)
        ds.SetGeoTransform((s.ll.X, s.cw, 0, s.ll.Y + s.ch * s.arr.shape[0], 0, -s.ch))
        ds.GetRasterBand(1).WriteArray(s.arr); ds = None
def NumPyArrayToRaster(arr, ll, cw, ch, value_to_nodata=None): return _NPRaster(arr, ll, cw, ch)
def _define(path, sr):
    from osgeo import gdal, osr
    ds = gdal.Open(path, gdal.GA_Update); o = osr.SpatialReference(); o.ImportFromEPSG(sr.factoryCode)
    ds.SetProjection(o.ExportToWkt()); ds = None
def _copy(src, dst):
    from osgeo import gdal; gdal.Translate(dst, src)
_MG.DefineProjection = staticmethod(_define)
_MG.CopyRaster = staticmethod(_copy)
PROGRESS = []
def SetProgressor(*a): PROGRESS.append(("set", a[1] if len(a) > 1 else ""))
def SetProgressorPosition(v): PROGRESS.append(("pos", v))
def ResetProgressor(): PROGRESS.append(("reset",))
class _Field:
    def __init__(s, name, type_="String"): s.name = name; s.type = type_
def ListFields(x):
    d = INPUTS[x]; keys = list((d.get("attrs") or [{}])[0].keys())
    return [_Field("OBJECTID", "OID"), _Field("Shape", "Geometry")] + [_Field(k) for k in keys]
