#!/usr/bin/python3
# ROS specific imports
import http.server
import socketserver
import std_msgs
import os
from string import Template

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


from .requesthdl3 import *

class MyServer(socketserver.TCPServer):
    allow_reuse_address = True

class MyRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            ButtonServer.global_server.buildPage()
            self.wfile.write(ButtonServer.global_server.page.encode('utf-8'))
            #self.wfile.close()
            return
        elif self.path == "/lib/jquery-3.6.4.min.js":
            http.server.SimpleHTTPRequestHandler.do_GET(self)
            return
        found = False
        for h in ButtonServer.repository.gethdl:
            if h.match(self.path):
                found = True
                h.run(self)
                break
        if not found:
            self.send_error(404, "requested path not available")

    def do_POST(self):
        if self.path == "/button":
            length = int(self.headers.get("content-length"))
            l = self.rfile.read(length).decode().split("=")
            if l[0] != "name":
                self.send_error(404,"Invalid form field")
                return
            buttonname = l[1]
            result="<html><body> Published: <b>%s</b> </body></html>" % buttonname
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(result.encode('utf-8')) 
            ButtonServer.global_server.publish(buttonname)
        return



class Button:
    def __init__(self,name,text,color,style=''):
        self.name = name.lower()
        self.text = text
        self.color = color
        self.style = 'width:300px;height:100px;font:24px Arial;'+style
        if self.color!="":
            self.style=self.style+('background-color:%s;' % self.color)

    def getInput(self):
        style='style="%s"' % self.style
        return """
        <p>
            <input class="%s" type="button" value="%s" %s/> 
            <br/>
        </p>
        """ % (self.name,self.text,style)

    def getFunction(self):
        T = Template("""
        var result = $$("#result");
        $$("input.${name}").click(function () {
            // alert("Button ${name} pressed");
            $$.post("button", {name:"${name}"}, function(xml) {
                    // alert("${name}: Received '"+xml+"'");
                    result.html(xml);
                });
            setTimeout(function () {result.html("_");}, 2000);
        });
        """)
        return T.substitute(name=self.name)


class ButtonServer(Node):
    global_server = None
    repository = HandlerRepository()
    def on_shutdown(self):
        self.httpd.socket.close()

    def __init__(self):
        super().__init__('button_server')
        ButtonServer.global_server = self
        self.handler = MyRequestHandler
        # rospy.on_shutdown(self.on_shutdown)
        self.pub = self.create_publisher(String, 'buttons', 1)
        self.declare_parameter('port', 5180)
        self.declare_parameter('root', '.')
        self.declare_parameter('num_buttons', 0)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.root = self.get_parameter('root').get_parameter_value().string_value
        nb = self.get_parameter('num_buttons').get_parameter_value().integer_value
        self.blist = []
        for i in range(nb):
            self.declare_parameter('button%d'%i, '')
            self.declare_parameter('button%d_text'%i, 'button%d'%i)
            self.declare_parameter('button%d_color'%i, '')
            self.declare_parameter('button%d_style'%i, '')
        for i in range(nb):
            bname = self.get_parameter("button%d"%i).get_parameter_value().string_value
            btext = self.get_parameter("button%d_text"%i).get_parameter_value().string_value
            bcolor = self.get_parameter("button%d_color"%i).get_parameter_value().string_value
            bstyle = self.get_parameter("button%d_style"%i).get_parameter_value().string_value
            if bname != "":
                self.blist.append(Button(bname,btext,bcolor,bstyle))
        if len(self.blist)==0:
            self.blist = [Button("test","Debug Button","lightgreen")]

        self.buildPage()

        os.chdir(self.root)
        self.httpd = MyServer(("", self.port), self.handler)

    def buildPage(self):
        self.page = """
        <http>
          <head>
          <meta http-equiv="content-type" content="text/html; charset=windows-1250">
          <META HTTP-EQUIV="PRAGMA" CONTENT="NO-CACHE">
          <META HTTP-EQUIV="CACHE-CONTROL" CONTENT="NO-CACHE">
          <META HTTP-EQUIV="refresh" CONTENT="15">
          <title>Button Server</title>
            <script language="javascript" type="text/javascript" src="lib/jquery-3.6.4.min.js"></script>
          </head>
          <body>
            <center>
            <h1>Button Server</h1>
            </center>
            %s
            <center>
            %s
            <br>
            <div id=result> _ </div>
            </center>
            <script id="source" language="javascript" type="text/javascript">
            $(document).ajaxError(function(e, xhr, settings, exception) {
                alert('error in: ' + settings.url + ' \\n'+'error:\\n' + exception + '\\nresponse:\\n' + xhr.responseText );
            });
            %s 
            </script>
          </body>
        </http>
        """ % (self.getHeader(), "<hl/>\n".join([b.getInput() for b in self.blist]),
                "\n".join([b.getFunction() for b in self.blist]))

    def getHeader(self):
        # To be overloaded
        return ""

    def publish(self,text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def run(self):
        self.get_logger().info("serving '%s' at port %d" % (self.root,self.port))
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.05)
            try:
                # rospy.loginfo("Handling next request")
                self.httpd.handle_request()
            except ValueError:
                if rclpy.ok():
                    raise

