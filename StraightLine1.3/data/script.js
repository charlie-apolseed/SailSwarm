// // Get current sensor readings when the page loads  
// window.addEventListener('load', getReadings);

// Create Heading Gauge
var headingGauge = new RadialGauge({
  renderTo: 'heading-gauge',
  minValue: 0,
  maxValue: 360,
  majorTicks: [
      "N",
      "NE",
      "E",
      "SE",
      "S",
      "SW",
      "W",
      "NW",
      "N"
  ],
  minorTicks: 22,
  ticksAngle: 360,
  startAngle: 180,
  strokeTicks: false,
  highlights: false,
  colorPlate: "#222",
  colorMajorTicks: "#f5f5f5",
  colorMinorTicks: "#ddd",
  colorNumbers: "#ccc",
  colorNeedle: "rgba(240, 128, 128, 1)",
  colorNeedleEnd: "rgba(255, 160, 122, .9)",
  valueBox: false,
  valueTextShadow: false,
  colorCircleInner: "#fff",
  colorNeedleCircleOuter: "#ccc",
  needleCircleSize: 15,
  needleCircleOuter: false,
  animationRule: "linear",
  needleType: "line",
  needleStart: 75,
  needleEnd: 99,
  needleWidth: 3,
  borders: true,
  borderInnerWidth: 0,
  borderMiddleWidth: 0,
  borderOuterWidth: 10,
  colorBorderOuter: "#ccc",
  colorBorderOuterEnd: "#ccc",
  colorNeedleShadowDown: "#222",
  borderShadowWidth: 0,
  animationDuration: 500
}).draw();
  

// Function to post updated wind direction
// Function to post updated wind direction
function updateWindDirection() {
  var newWindDir = document.getElementById('new-wind-dir').value;
  if (newWindDir === "" || isNaN(newWindDir) || newWindDir < 0 || newWindDir > 360) {
    alert("Please enter a valid wind direction between 0 and 360 degrees.");
    return;
  }

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/windDir?windDir=" + newWindDir, true);
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      console.log("Wind direction updated successfully.");
    }
  };
  xhr.send();
}

// // Function to get current readings on the webpage when it loads for the first time
// function getReadings(){
//   var xhr = new XMLHttpRequest();
//   xhr.onreadystatechange = function() {
//     if (this.readyState == 4 && this.status == 200) {
//       var myObj = JSON.parse(this.responseText);
//       console.log(myObj);
//       var heading = myObj.heading;
//       var windDir = myObj.windDir;
//       var targetDir = myObj.targetDir;
//       headingGauge.value = heading;
     
      
//       // Update heading value display
//       document.getElementById('heading-value').innerHTML = "<span>" + heading + "</span>";
      
//       // Update wind direction value display
//       document.getElementById('windDirection-value').innerHTML = "<span>" + windDir + "</span>";

//       // Update target direction value display
//       document.getElementById('targetDirection-value').innerHTML = "<span>" + targetDir + "</span>";
//     }
//   }; 
//   xhr.open("GET", "/readings", true);
//   xhr.send();
// }

if (!!window.EventSource) {
  var source = new EventSource('/events');
  
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);
  
  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);
  
  source.addEventListener('new_readings', function(e) {
    console.log("new_readings", e.data);
    var myObj = JSON.parse(e.data);
    console.log(myObj);
    headingGauge.value = myObj.heading;
    
    
    // Update heading value display
    document.getElementById('heading-value').innerHTML = "<span> Current Heading: " + myObj.heading + "&deg;</span>";
    
    // Update wind direction value display
    document.getElementById('windDirection-value').innerHTML = "<span>" + myObj.windDir + "&deg;</span>";

    // Update target direction value display
    document.getElementById('targetHeading-value').innerHTML = "<span> Target Heading: " + myObj.targetHeading + "&deg;</span>";

    // Update heading tolerance value display
    document.getElementById('headingTolerance-value').innerHTML = "<span> Heading Tolerance: " + myObj.headingTolerance + "&deg;</span>";

    // Update current tack value display
    document.getElementById('currentTack-value').innerHTML = "<span> Current Tack: " + myObj.currentTack + "</span>";

    // Update current action value display
    document.getElementById('currentAction-value').innerHTML = "<span> Current Action: " + myObj.currentAction + "</span>";
  }, false);
}
