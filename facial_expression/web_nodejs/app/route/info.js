var express = require('express');
var router = express.Router();
var degree = -20;

/* GET home page. */
router.get('/', function(req, res, next) {
  res.writeHead(200, {'Content-Type': 'text/plain'});
  res.write(degree);
  res.end();
});

router.post('/', function(req, res, next) {
  degree = req.body.degree;
  console.log("POSTED: ", degree);
});

module.exports = router;
