
const multer = require('multer');
const upload = multer({ dest: './uploads/' });


const express = require('express');
const app = express();
const fs = require('fs');
const p = require('process');

const { spawn } = require('child_process');

app.use(function (err, req, res, next) {
  console.log('This is the invalid field ->', err.field)
  next(err)
})

function recog(fn) {
  const ls = spawn('python', 
  [
    __dirname + '/cmp.py',
    __dirname + '/' + fn
  ]);

  ls.stdout.on('data', (data) => {
    // res.text(data);
    console.log(`stdout: ${data}`);
  });

  ls.stderr.on('data', (data) => {
    console.error(`stderr: ${data}`);
  });

  ls.on('close', (code) => {
    console.log(`child process exited with code ${code}`);
  });

}

app.route('/').get((req, res) => {
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.write('<form action="target" method="post" enctype="multipart/form-data">');
  res.write('<input type="file" name="file"><br>');
  res.write('<input type="submit">');
  res.write('</form>');
  return res.end();
});

app.post('/target', upload.single('file'), (req, res) => { 
  console.log(`new upload = ${req.file.filename}\n`);
  console.log(req.file);
  

});


app.post('/upload', upload.single('file'), (req, res) => { 
  //console.log(`new upload = ${req.file.filename}\n`);
  ///console.log(req.file);
  
  const ls = spawn('python', 
  [
    __dirname + '/cmp.py',
    __dirname + '/' + req.file.path
  ]);

  ls.stdout.on('data', (data) => {
    res.send(data);
    console.log(`MATCH: ${data}`);
  });

  ls.stderr.on('data', (data) => {
    console.error(`stderr: ${data}`);
  });

  ls.on('close', (code) => {
    // console.log(`child process exited with code ${code}`);

    fs.unlink(req.file.path, () => {})
  });
});

app.listen(8888);