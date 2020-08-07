$(function(){
	
	var currentValue = $('#currentValue');
	
	$("#slider").slider({ 
		max: 255,
		min: 1,
		value:180,
		slide: function(event, ui) {
			currentValue.html(ui.value);
		}
	});
	
});
