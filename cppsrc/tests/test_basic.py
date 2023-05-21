import pyCoverageControl as m


def test_main():
    assert m.__version__ == "0.2.0"
    pt = m.Point2(4,3)
    assert m[0] == 4
    assert m[1] == 3
