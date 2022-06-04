from typing import List

from sensor_msgs.msg import PointCloud2, PointField
from numpy.lib.recfunctions import structured_to_unstructured
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation

# sizes (in bytes) of PointField types
_POINT_FIELDS_SIZES = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8
}
# mappings between PointField types and numpy types
_POINT_FIELDS_AND_NUMPY_DTYPES_PAIRS = [
    (PointField.INT8, np.dtype('int8')),
    (PointField.UINT8, np.dtype('uint8')),
    (PointField.INT16, np.dtype('int16')),
    (PointField.UINT16, np.dtype('uint16')),
    (PointField.INT32, np.dtype('int32')),
    (PointField.UINT32, np.dtype('uint32')),
    (PointField.FLOAT32, np.dtype('float32')),
    (PointField.FLOAT64, np.dtype('float64'))
]

_POINT_FIELD_TO_NUMPY_DTYPE = dict(_POINT_FIELDS_AND_NUMPY_DTYPES_PAIRS)

_DUMMY_FIELD_PREFIX = '__'

def transformation_stamped_to_transformation_matrix(transformation_stamped: TransformStamped):
    trans_matrix = np.eye(4, dtype=np.float32)
    trans_matrix[:3, :3] = Rotation.from_quat([
        transformation_stamped.transform.rotation.x,
        transformation_stamped.transform.rotation.y,
        transformation_stamped.transform.rotation.z,
        transformation_stamped.transform.rotation.w
    ]).as_matrix()
    trans_matrix[:3, 3] = np.array([
        transformation_stamped.transform.translation.x,
        transformation_stamped.transform.translation.y,
        transformation_stamped.transform.translation.z,
    ])

    return trans_matrix

def _point_fields_to_numpy_dtypes(fields, point_step):
    """
        Convert a list of PointFields to a numpy record datatype.
    """
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (_DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = _POINT_FIELD_TO_NUMPY_DTYPE[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += _POINT_FIELDS_SIZES[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (_DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def ros_point_cloud_to_numpy_point_cloud(point_cloud: PointCloud2, field_names: List[str] = None) -> np.array:
    if field_names is None:
        field_names = ['x', 'y', 'z', 'intensity']
    dtype_list = _point_fields_to_numpy_dtypes(point_cloud.fields, point_cloud.point_step)
    point_cloud = np.frombuffer(point_cloud.data, dtype_list)
    point_cloud = structured_to_unstructured(point_cloud[field_names]).copy()
    point_cloud = point_cloud.astype(np.float32)
    return point_cloud
