import types

def CreateMethodListDecorator():
    class MethodListDecorator(object):
        methods = list()

        def __init__(self, func):
            self._func = func
            self.__class__.methods.append(func)

        def __get__(self, obj, type=None):
            return self.__class__(self._func.__get__(obj, type))

        def __call__(self, *args, **kw_args):
            self._func(*args, **kw_args)

        @classmethod
        def Bind(cls, instance):
            for method in cls.methods:
                bound_method = types.MethodType(method, instance, type(instance))
                setattr(instance, method.__name__, bound_method)

    return MethodListDecorator
