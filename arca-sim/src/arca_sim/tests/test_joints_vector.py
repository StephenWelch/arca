import pytest
import numpy as np
from ..joints_vector import JointsVector, JointsType

def test_integer_indexing():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    jv = JointsVector(JointsType.FULL, values)
    
    assert jv[0] == 1.0
    assert jv[3] == 4.0
    assert jv[-1] == 6.0

def test_slice_indexing():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    jv = JointsVector(JointsType.FULL, values)
    
    np.testing.assert_array_equal(jv[1:4], np.array([2.0, 3.0, 4.0]))
    np.testing.assert_array_equal(jv[:3], np.array([1.0, 2.0, 3.0]))
    np.testing.assert_array_equal(jv[3:], np.array([4.0, 5.0, 6.0]))

def test_string_indexing():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    names = ["l_hip_roll", "l_hip_pitch", "l_knee_pitch", 
             "r_hip_roll", "r_hip_pitch", "r_knee_pitch"]
    jv = JointsVector(JointsType.FULL, values, names)
    
    assert jv["l_hip_roll"] == 1.0
    assert jv["r_hip_pitch"] == 5.0

def test_list_string_indexing():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    names = ["l_hip_roll", "l_hip_pitch", "l_knee_pitch", 
             "r_hip_roll", "r_hip_pitch", "r_knee_pitch"]
    jv = JointsVector(JointsType.FULL, values, names)
    
    # Test string list indexing
    np.testing.assert_array_equal(
        jv[["l_hip_roll", "r_hip_roll", "r_knee_pitch"]],
        np.array([1.0, 4.0, 6.0])
    )

def test_error_cases():
    values = np.array([1.0, 2.0, 3.0])
    jv = JointsVector(JointsType.FULL, values)
    
    # Test index out of bounds
    with pytest.raises(IndexError):
        _ = jv[3]
    
    # Test invalid key type
    with pytest.raises(ValueError):
        _ = jv[1.5]  # Float indices not supported
    
    # Test string indexing without names
    with pytest.raises(ValueError):
        _ = jv["l_hip_roll"]

def test_setitem():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    names = ["l_hip_roll", "l_hip_pitch", "l_knee_pitch", 
             "r_hip_roll", "r_hip_pitch", "r_knee_pitch"]
    jv = JointsVector(JointsType.FULL, values, names)
    
    # Test integer indexing
    jv[0] = 10.0
    assert jv[0] == 10.0
    
    # Test string indexing
    jv["l_hip_pitch"] = 20.0
    assert jv["l_hip_pitch"] == 20.0
    
    # Test slice indexing
    jv[3:5] = [40.0, 50.0]
    np.testing.assert_array_equal(jv[3:5], np.array([40.0, 50.0])) 