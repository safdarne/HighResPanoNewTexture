
precision mediump float;

varying mediump vec2 coordinate;
uniform sampler2D videoframe;

void main( )
{
	//vec4 color = texture2D( videoframe, coordinate );
    // instead of flipping the image in the last rendering stage to the screen, flip it in the shader
    vec4 color = texture2D( videoframe, vec2( coordinate.x, 1.0 - coordinate.y ) );
    
	//gl_FragColor.bgra = vec4( color.b, 0.0 * color.g, color.r, color.a );
    gl_FragColor.bgra = vec4( color.b, color.g, color.r , color.a );
    
}