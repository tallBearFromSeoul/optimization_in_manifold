class Error(Exception):
    pass
class NonSquareMatrix(Error):
    pass
class NonSymmetricMatrix(Error):
    pass

class NotNumpyArray(Error):
    pass
class NotOneDimArray(Error):
    pass
class NotTwoDimArray(Error):
    pass

class Not3Vector(Error):
    pass
class Not4Vector(Error):
    pass
class Not3by3Matrix(Error):
    pass

class NotPositive(Error):
    pass
class NotInt(Error):
    pass
class NotFloat(Error):
    pass


