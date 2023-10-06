
import re
import urllib.parse

class requesthdl:
    def __init__(self,path_re):
        self.path_re = path_re

    def match(self, path):
        return re.match(self.path_re,path)

    def parse(self, path):
        argpos = path.find("?")
        args = {}
        if argpos > 0 :
            args=dict([s.split("=") for s in path[argpos+1:].split("&")])
            for k in list(args.keys()):
                args[k] = urllib.parse.unquote(args[k])
        return args

    def run(self, request):
        # To be overloaded
        request.send_error(404, "request handler not implemented")
        

class HandlerRepository:
    def __init__(self):
        self.gethdl = []
        self.posthdl = []

    def addGetHandler(self,hdl):
        self.gethdl.append(hdl)

    def addPostHandler(self,hdl):
        self.posthdl.append(hdl)



