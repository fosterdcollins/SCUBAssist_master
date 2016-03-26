MS = 3
BS = .3125

Top = [[-MS/2, MS/2, 0]', [MS/2, MS/2, 0]', [MS/2, -MS/2, 0]', [-MS/2, -MS/2, 0]'];
Side2 = [[-MS/2, MS/2+BS, -BS]', [MS/2, MS/2+BS, -BS]', [MS/2, MS/2+BS, -BS-MS]', [-MS/2, MS/2+BS, -BS-MS]'];
Side3 = [[-MS/2-BS, -MS/2, -BS]', [-MS/2-BS, MS/2, -BS]', [-MS/2-BS, MS/2, -BS-MS]', [-MS/2-BS, -MS/2, -BS-MS]'];
Side4 = [[-MS/2, -MS/2-BS, -BS]', [MS/2, -MS/2-BS, -BS]', [MS/2, -MS/2-BS, -BS-MS]', [-MS/2, -MS/2-BS, -BS-MS]'];
Side5 = [[MS/2+BS, -MS/2, -BS]', [MS/2+BS, MS/2, -BS]', [MS/2+BS, MS/2, -BS-MS]', [MS/2+BS, -MS/2, -BS-MS]'];

points = [Top, Side2, Side3, Side4, Side5]

plot3(points(1,:), points(2,:), points(3,:), 'k.')