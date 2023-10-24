import requests
#
#multipart_form_data = {
#    'file2': ('custom_file_name.zip', open('send/target.jpg', 'rb')),
#    'action': (None, 'store'),
#    'path': (None, '/path1')
#}
#
#response = requests.post('http://localhost:8000', files=multipart_form_data)
with open('send/target.jpg', 'rb') as f:
  r = requests.post('http://localhost:8888/upload', files={'file': f})
  print(r.content)