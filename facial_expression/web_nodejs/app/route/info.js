var express = require('express');
var router = express.Router();
var degree = -20;

/* GET home page. */
router.get('/', function(req, res, next) {
  res.writeHead(200, {'Content-Type': 'text/plain'});
  res.write(mode + ",");
  res.write(degree);
  res.end();
});

router.post('/', function(req, res, next) {
  degree = req.body.degree;
  mode = req.body.mode;
  console.log("POSTED MODE: ", mode);
  console.log("POSTED DEGREE:", degree);
});

module.exports = router;
