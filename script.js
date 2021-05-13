var counter = 0;

function reloadAllImages(imgs) {
    counter++;
    document.getElementById('camera').src='thermocam-current.jpg?c=' + counter;
    imgs.forEach((img) => {
        let div = document.getElementById(img.name);
        if (!div) {
            div = document.createElement("div");
            div.setAttribute("id", img.name)
            div.innerHTML = `<h3>${img.title}</h3> <a href='${img.name}.tiff'>raw</a><img/><div/>`;
            document.getElementById('webimgs').appendChild(div);
        }
        div.getElementsByTagName('img')[0].src = `${img.name}.jpg?c=${counter}`;
        div.getElementsByTagName('div')[0].textContent = img.desc;
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