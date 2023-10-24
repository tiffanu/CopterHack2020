from http.server import HTTPServer, BaseHTTPRequestHandler
import face_recognition

from io import BytesIO
import os
import cgi
import shutil



class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):

      # Load the jpg files into numpy arrays
      target_image = face_recognition.load_image_file("target.jpg")
      unknown_image = face_recognition.load_image_file("barack.jpg")

      # Get the face encodings for each face in each image file
      # Since there could be more than one face in each image, it returns a list of encodings.
      # But since I know each image only has one face, I only care about the first encoding in each image, so I grab index 0.
      try:
        target_face_encoding = face_recognition.face_encodings(target_image)[0]
        unknown_face_encoding = face_recognition.face_encodings(unknown_image)[0]

      except IndexError:
        print("I wasn't able to locate any faces in at least one of the images. Check the image files. Aborting...")
        quit()

      known_faces = [
          target_face_encoding,
      ]

      # results is an array of True/False telling if the unknown face matched anyone in the known_faces array
      results = face_recognition.compare_faces(known_faces, unknown_face_encoding)

      print("Is the unknown face a picture of Biden? {}".format(results[0]))
      #print("Is the unknown face a picture of Obama? {}".format(results[1]))
      print("Is the unknown face a new person that we've never seen before? {}".format(not True in results))


      self.send_response(200)
      self.end_headers()
      self.wfile.write(b'Recognition i')



    def send_headers(self):
        npath = os.path.normpath(self.path)
        npath = npath[1:]
        path_elements = npath.split('/')

        if path_elements[0] == "f":
            reqfile = path_elements[1]

            if not os.path.isfile(reqfile) or not os.access(reqfile, os.R_OK):
                self.send_error(404, "file not found")
                return None

            content, encoding = memetypes.MimeTypes().guess_type(reqfile)
            if content is None:
                content = "application/octet-stream"

            info = os.stat(reqfile)

            self.send_response(200)
            self.send_header("Content-Type", content)
            self.send_header("Content-Encoding", encoding)
            self.send_header("Content-Length", info.st_size)
            self.end_headers()

        elif path_elements[0] == "upload":
            self.send_response(200)
            self.send_header("Content-Type", "text/json; charset=utf-8")
            self.end_headers()

        else:
            self.send_error(404, "fuck")
            return None

        return path_elements


    def do_GET(self):
        elements = self.send_headers()
        if elements is None:
            return

        reqfile = elements[1]
        f = open(reqfile, 'rb')
        shutil.copyfileobj(f, self.wfile)
        f.close()


    def do_POST(self):
        elements = self.send_headers()
        if elements is None or elements[0] != "upload":
            return

        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={
                "REQUEST_METHOD": "POST",
                "CONTENT_TYPE":   self.headers['Content-Type']
            })

        _, ext = os.path.splitext(form["file"].filename)


        fdst = open("/home/kberezin/Documents/code/copterhack/recog-server/test.jpg", "wb")
        shutil.copyfileobj(form["file"].file, fdst)
        fdst.close()

        result = {
            "data": { "url": SITE_ROOT + "/f/" + fname },
            "success": True,
            "status": 200,
        }

        self.wfile.write(json.dumps(result))

httpd = HTTPServer(('localhost', 8000), SimpleHTTPRequestHandler)
httpd.serve_forever()