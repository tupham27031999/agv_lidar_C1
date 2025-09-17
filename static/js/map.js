document.addEventListener('DOMContentLoaded', function() {
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');

    // Tạo một đối tượng ảnh để làm nền
    const backgroundImage = new Image();
    let pointsToDraw = [];

    /**
     * Vẽ một điểm và tên của nó lên canvas
     * @param {object} point - Đối tượng điểm chứa tọa độ, màu sắc, kích thước, tên
     */
    function drawPoint(point) {
        // Chuyển đổi màu từ mảng [R, G, B] sang chuỗi CSS
        const color = `rgb(${point.color[0]}, ${point.color[1]}, ${point.color[2]})`;

        // Vẽ điểm (hình tròn)
        ctx.beginPath();
        ctx.arc(point.toa_do_x, point.toa_do_y, point.kich_thuoc, 0, 2 * Math.PI, false);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.lineWidth = 1;
        ctx.strokeStyle = '#003300';
        ctx.stroke();

        // Vẽ tên điểm
        ctx.font = '12px Arial';
        ctx.fillStyle = 'black';
        ctx.fillText(point.ten_diem, point.toa_do_x + 10, point.toa_do_y + 4);

        // Vẽ mũi tên chỉ hướng cho AGV
        if (point.ten_diem === 'AGV') {
            drawArrow(point.toa_do_x, point.toa_do_y, point.goc, point.kich_thuoc + 10, 'yellow');
        }
    }

    /**
     * Vẽ mũi tên chỉ hướng
     */
    function drawArrow(x, y, angle, length, color) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(angle);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(length, 0);
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.stroke();
        // Đầu mũi tên
        ctx.moveTo(length, 0);
        ctx.lineTo(length - 8, -5);
        ctx.moveTo(length, 0);
        ctx.lineTo(length - 8, 5);
        ctx.stroke();
        ctx.restore();
    }

    /**
     * Hàm chính để vẽ lại toàn bộ canvas
     */
    function redrawCanvas() {
        // Xóa toàn bộ canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // Vẽ lại ảnh nền
        ctx.drawImage(backgroundImage, 0, 0, canvas.width, canvas.height);
        // Vẽ lại tất cả các điểm
        pointsToDraw.forEach(drawPoint);
    }

    /**
     * Lấy dữ liệu điểm mới từ server và yêu cầu vẽ lại
     */
    async function updatePoints() {
        try {
            const response = await fetch('/get_points_to_draw');
            pointsToDraw = await response.json();
            redrawCanvas();
        } catch (error) {
            console.error('Lỗi khi cập nhật điểm:', error);
        }
    }

    // Bắt đầu quá trình:
    // 1. Đặt nguồn cho ảnh nền.
    backgroundImage.src = '/img_none_all';
    // 2. Khi ảnh đã tải xong, thực hiện lần cập nhật đầu tiên và đặt lịch cập nhật định kỳ.
    backgroundImage.onload = () => {
        updatePoints(); // Cập nhật lần đầu
        setInterval(updatePoints, 2000); // Cập nhật mỗi 2 giây
    };

    // --- PHẦN THÊM MỚI: KẾT NỐI CÁC THÀNH PHẦN ĐIỀU KHIỂN ---

    const tienMaxSlider = document.getElementById('tien_max_slider');
    const tienMaxValue = document.getElementById('tien_max_value');
    const reMaxSlider = document.getElementById('re_max_slider');
    const reMaxValue = document.getElementById('re_max_value');

    /**
     * Gửi yêu cầu cập nhật cài đặt lên server
     * @param {string} setting - Tên cài đặt (e.g., 'tien_max')
     * @param {any} value - Giá trị mới
     */
    async function sendSettingUpdate(setting, value) {
        try {
            const response = await fetch('/update_setting', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ setting, value }),
            });
            const result = await response.json();
            if (response.ok) {
                console.log('Cập nhật thành công:', result.message);
            } else {
                console.error('Lỗi khi cập nhật:', result.message);
            }
        } catch (error) {
            console.error('Lỗi mạng khi gửi cập nhật:', error);
        }
    }

    // Lắng nghe sự kiện thay đổi trên thanh trượt vận tốc tiến
    tienMaxSlider.addEventListener('input', (event) => {
        // Cập nhật giá trị hiển thị ngay lập tức khi kéo
        tienMaxValue.textContent = event.target.value;
    });
    // Gửi giá trị lên server khi người dùng thả chuột (sự kiện 'change')
    tienMaxSlider.addEventListener('change', (event) => {
        sendSettingUpdate('tien_max', event.target.value);
    });

    // Lắng nghe sự kiện thay đổi trên thanh trượt vận tốc rẽ
    reMaxSlider.addEventListener('input', (event) => {
        reMaxValue.textContent = event.target.value;
    });
    reMaxSlider.addEventListener('change', (event) => {
        sendSettingUpdate('re_max', event.target.value);
    });
});