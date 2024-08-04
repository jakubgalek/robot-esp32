document.addEventListener('DOMContentLoaded', (event) => {
    // Get the slider container, button, and value display elements
    const slideContainer = document.getElementById('slideContainer');
    const showSliderButton = document.getElementById('showSliderButton');
    const slider = document.getElementById('myRange');
    const valueDisplay = document.getElementById('value');

    // Function to update the displayed value
    function updateValue() {
        valueDisplay.textContent = slider.value;
    }

    // Update the value display when the slider value changes
    slider.addEventListener('input', updateValue);

    // Handle button click to toggle the slider
    showSliderButton.addEventListener('click', function() {
        if (window.getComputedStyle(slideContainer).display === 'none') {
            // Show the slider container
            slideContainer.style.display = 'block';
            showSliderButton.textContent = 'Ukryj'; // Change button text
        } else {
            // Hide the slider container
            slideContainer.style.display = 'none';
            showSliderButton.textContent = 'Pokaż prędkość'; // Change button text
        }
    });

    // Initialize the display value
    updateValue();

    // Ensure button text reflects initial state on page load
    if (window.getComputedStyle(slideContainer).display === 'none') {
        showSliderButton.textContent = 'Pokaż prędkość'; // For mobile
    } else {
        showSliderButton.textContent = 'Ukryj'; // For desktop
    }
});
