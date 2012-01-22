var pos = [0, 0, 0];

function initShaders(gl) {
    var fragmentShader = getShader(gl, "shader-fs");
    if (! fragmentShader) {
        return;
    }
    var vertexShader = getShader(gl, "shader-vs");
    if (! vertexShader) {
        return;
    }
  
    var shader = gl.createProgram();
    gl.attachShader(shader, vertexShader);
    gl.attachShader(shader, fragmentShader);
    gl.linkProgram(shader);
  
    if (!gl.getProgramParameter(shader, gl.LINK_STATUS)) {
        alert("Unable to link the shader program: " + gl.getProgramInfoLog(shader));
        return null;
    }
  
    gl.useProgram(shader);
    shader.aVertexPosition = gl.getAttribLocation(shader, "aVertexPosition");
    shader.uTime = gl.getUniformLocation(shader, "uTime");
    shader.uPos = gl.getUniformLocation(shader, "uPos");
    shader.uDir = gl.getUniformLocation(shader, "uDir");
    return shader;
}

function getShader(gl, id) {
    var shaderScript = document.getElementById(id);
    var source = shaderScript.firstChild.nodeValue;
 
    var shader = null;
    if (shaderScript.type == "x-shader/x-fragment") {
        shader = gl.createShader(gl.FRAGMENT_SHADER);
    } else if (shaderScript.type == "x-shader/x-vertex") {
        shader = gl.createShader(gl.VERTEX_SHADER);
    }
  
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
  
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        alert("An error occurred compiling the shaders: " + gl.getShaderInfoLog(shader));
        return null;
    }
  
    return shader;
}

function loadTexture(gl, textureUrl, callBack) {
    var texture = gl.createTexture();
    var image = new Image();
    image.onload = function() {
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.bindTexture(gl.TEXTURE_2D, null);
        callBack();
    };
    image.src = textureUrl;
    return texture;
}

onload = function() {
    var canvas = document.getElementById("canvas");
    var gl = canvas.getContext("experimental-webgl");

    onload.fps = document.createElement("p");
    onload.fps.appendChild(document.createTextNode("...initializing..."));
    canvas.parentNode.appendChild(onload.fps);

    /* initialize shaders. */
    var shader = initShaders(gl);
    if (! shader) {
        return;
    }

    effectInit(gl, shader);
}

var effectPos = function(x, y, z) {
    pos = [x, y, z];
}

var effectStart = function(gl, shader) {
    /* Initialize the quad required for world rendering */
    var quadVertex = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, quadVertex);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
         1.0,  1.0, 0.0,
         1.0, -1.0, 0.0,
        -1.0,  1.0, 0.0,
        -1.0, -1.0, 0.0
    ]), gl.STATIC_DRAW);

    var forward = false;
    var speed = 0;
    var mouseX = 0.5;
    var mouseY = 0.5;

    var canvas = document.getElementById("canvas");
    canvas.onmousemove = function(event) {
        mouseX = event.pageX / this.width;
        mouseY = event.pageY / this.height;
    };

    canvas.onmousedown = function() {
	speed = 0;
        forward = true;
    };
    canvas.onmouseup = function() {
        forward = false;
    };

    var lastTime = 0;
    var startTime = Date.now();
    var fpsNumber = 0;
    setInterval(function() {
        var oldLastTime = lastTime;
        lastTime = (Date.now() - startTime) / 1000;
        var timeStep = lastTime - oldLastTime;

        gl.uniform1f(shader.uTime, lastTime);
        var dz = Math.cos(mouseY * Math.PI);
        var dz2 = Math.sin(mouseY * Math.PI);
        var dx = Math.sin(mouseX * Math.PI * 2) * dz2;
        var dy = Math.cos(mouseX * Math.PI * 2) * dz2;
        if (forward) {
            pos[0] += dx * timeStep * speed;
            pos[1] += dy * timeStep * speed;
            pos[2] += dz * timeStep * speed;
	    speed += timeStep;
	    if (speed > 1.0) {
		speed = 1.0;
	    }
	}
        gl.uniform3f(shader.uPos, pos[0], pos[1], pos[2]);
        gl.uniform3f(shader.uDir, dx, dy, dz);
        onload.fps.removeChild(onload.fps.firstChild);
        onload.fps.appendChild(document.createTextNode(Math.round(fpsNumber / lastTime) + " fps"));
        fpsNumber ++;

        gl.bindBuffer(gl.ARRAY_BUFFER, quadVertex);
        gl.vertexAttribPointer(shader.aVertexPosition, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(shader.aVertexPosition);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
        gl.flush();
    }, 1000 / 60);
}
