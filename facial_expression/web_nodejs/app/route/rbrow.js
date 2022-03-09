var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/', function(req, res, next) {
	res.render('rbrow', { degree: -20, reload: 0});
	// console.log(req.query.reload);
	// res.render('rbrow', {degree: req.body.degree, reload: req.query.reload});
});

/* MOVE right eyebrow */
router.post('/', function(req, res, next) {
	// var deg = req.body.degree;
	// console.log("POSTED: ", deg);
	// var degree = -20;
	// res.render('rbrow', {degree: -20, reload: 1});
	// res.render('rbrow', {degree: -20, reload: 1}, function (err, html) {
	// 	res.send(html)
	// })
	res.redirect('https//www.google.com/?hl=ja');
});

module.exports = router;

