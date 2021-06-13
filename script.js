var counter = 0;

function toggleUpdate(img) {
    img.parentElement.getElementsByTagName('input')[0].checked ^= 1;
}

function reloadAllImages(imgs) {
    counter++;
    document.getElementById('camera').src='thermocam-current.jpg?c=' + counter;
    let x = 1;
    webimgs = document.getElementById('webimgs');
    imgs.forEach((img_list) => {
	let y = 1;
	img_list.forEach((img) => {
            let div = document.getElementById(img.name);
            if (!div) {
		div = document.createElement("div");
		div.setAttribute("id", img.name)
		div.innerHTML = `<h3>${img.title}</h3> <a href='${img.name}.tiff'>raw</a><input type="checkbox" checked=true><img onclick="toggleUpdate(this)" /><div/>`;
		div.style.gridColumn = `${x}`;
		div.style.gridRow = `${y}`;
		webimgs.append(div);
            }
	    imgel = div.getElementsByTagName('img')[0];
	    if (div.getElementsByTagName('input')[0].checked) {
		imgel.src = `${img.name}.jpg?c=${counter}`;
		imgel.style.filter = "";
	    } else {
		imgel.style.filter = "grayscale(100%)";
	    }
            div.getElementsByTagName('div')[0].textContent = img.desc;
	    y++;
	});
	x++;
    });
};

var wsProtocol = 'ws://';
if (window.location.protocol === 'https:') {
    wsProtocol = 'wss://';
}

function reconnect() {
    var socket = new WebSocket(wsProtocol + location.host + "/ws");

    socket.onopen = ()=>{
        console.log('open');
    }

    socket.onclose = ()=>{
        console.log('close');
        setTimeout(() => { reconnect(); }, 3000);
    }

    socket.onmessage = (event) => {
        let msg = JSON.parse(event.data);
        reloadAllImages(msg.imgs);
    }
}
reconnect();
