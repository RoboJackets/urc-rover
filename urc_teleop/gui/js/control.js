// Joystick publication
// --------------------


// This function is to eliminate an error when integer value
// floats are passed into rosbridge
function checkValidity(num) {
    if (num.toFixed(2) == 1.0) {
        return 99.9;
    }
    else if (num.toFixed(2) == 0.0) {
        return 0.000001;
    }
    else if (num.toFixed(2) == -1.0) {
        return -99.9;
    }
    else {
        return num;
    }
}

const joy_publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/joy',
  messageType : 'sensor_msgs/msg/Joy'
});

function publishMovementInput(gamepad) {
    let joy_msg = new ROSLIB.Message({
        axes: [
            checkValidity(0.0),
            checkValidity(gamepad.axes[1]),
            checkValidity(0.0),
            checkValidity(0.0),
            checkValidity(gamepad.axes[3])
        ],
        buttons: [
            gamepad.buttons[0].pressed ? 1 : 0,
            gamepad.buttons[1].pressed ? 1 : 0,
            gamepad.buttons[2].pressed ? 1 : 0,
            gamepad.buttons[3].pressed ? 1 : 0
        ]
    });
    joy_publisher.publish(joy_msg)
}


// Controller/Joystick functionality
// -----------------------


window.addEventListener("gamepadconnected", event => {
    console.log('Gamepad connected!');
    console.log(event.gamepad);
});

window.addEventListener("gamepaddisconnected", event => {
    console.log('Gamepad disconnected!');
    console.log(event.gamepad);
});

const gamepadDisplay = document.getElementById('gamepad-display');
const joystickDisplay = document.getElementById('joystick-display');

// Checks the gamepad inputs every 17 ms, publishes inputs across rosbridge
setInterval(function() {
    var gamepad_list = navigator.getGamepads();
    
    // controller (rover movement)
    if (gamepad_list[0]) {
        const gamepadState = {
            id : gamepad_list[0].id,
            axes: [
                //left stick
                gamepad_list[0].axes[0].toFixed(2), // -1 left, 1 right
                gamepad_list[0].axes[1].toFixed(2),  // -1 up, 1 down

                //right stick
                gamepad_list[0].axes[2].toFixed(2), // -1 left, 1 right
                gamepad_list[0].axes[3].toFixed(2)  // -1 up, 1 down
            ],
            buttons: [
                { button_0 : gamepad_list[0].buttons[0].pressed },
                { button_1 : gamepad_list[0].buttons[1].pressed }, // increase vel
                { button_2 : gamepad_list[0].buttons[2].pressed },
                { button_3 : gamepad_list[0].buttons[3].pressed }  // decrease vel
            ]
        }
        gamepadDisplay.textContent = JSON.stringify(gamepadState, null, 2);
        publishMovementInput(gamepad_list[0]);
    }
    
    // joystick (arm movement)
    //if (gamepad_list[1]) {
    //    const joystickState = {
    //        id : gamepad_list[1].id,
    //        axes: [
    //            gamepad_list[1].axes[0].toFixed(2)
    //        ],
    //        buttons: [
    //            { button_0 : gamepad_list[1].buttons[0].pressed }
    //        ]
    //    }
    //    joystickDisplay.textContent = JSON.stringify(joystickState, null, 2);
        //publishArmInput(gamepad_list[1])
    //}
}, 17)