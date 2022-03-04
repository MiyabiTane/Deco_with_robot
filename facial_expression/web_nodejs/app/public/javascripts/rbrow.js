console.log("Hello");
main();

function main() {
	// Initialize
	var c = document.getElementById('webgl');
	c.width = 640;
	c.height = 480;
	var gl = c.getContext('webgl');
	// Set clear color to black, fully opaque
	gl.clearColor(0.0, 0.0, 0.0, 1.0);
	// Clear the color buffer with specified clear color
	gl.clear(gl.COLOR_BUFFER_BIT);
}
