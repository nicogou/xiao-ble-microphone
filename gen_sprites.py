from PIL import Image
import os

# (source png, output header, C symbol, size in px)
SPRITES = [
    ('ufo.png',       'ufo_img.h',       'ufo_img',       34),
    ('asteroid.png',  'asteroid_img.h',  'asteroid_img',  22),
    ('explosion.png', 'explosion_img.h', 'explosion_img', 90),
    ('confetti.png',  'confetti_img.h',  'confetti_img', 160),
]


def gen(src, header, sym, size):
    img = Image.open(src).convert('RGBA').resize((size, size), Image.LANCZOS)

    # LVGL 9 RGB565A8: all RGB565 pixels first, then all alpha values.
    rgb_data = []
    alpha_data = []
    for r, g, b, a in img.getdata():
        rgb = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        rgb_data += [rgb & 0xFF, (rgb >> 8) & 0xFF]
        alpha_data.append(a)

    data = rgb_data + alpha_data

    out = ['#pragma once', '#include <lvgl.h>', '']
    out += ['static const uint8_t ' + sym + '_map[] = {']
    for i in range(0, len(data), 12):
        out.append('    ' + ', '.join('0x{:02X}'.format(b) for b in data[i:i + 12]) + ',')
    out += ['};', '',
            'static const lv_image_dsc_t ' + sym + ' = {',
            '    .header = {',
            '        .magic      = LV_IMAGE_HEADER_MAGIC,',
            '        .cf         = LV_COLOR_FORMAT_NATIVE_WITH_ALPHA,',
            '        .flags      = 0,',
            '        .w          = ' + str(size) + ',',
            '        .h          = ' + str(size) + ',',
            '        .stride     = ' + str(size * 2) + ',',
            '        .reserved_2 = 0,',
            '    },',
            '    .data_size = ' + str(len(data)) + ',',
            '    .data      = ' + sym + '_map,',
            '};']

    path = os.path.join('app', 'src', header)
    with open(path, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print('OK: ' + path + '  (' + str(len(data)) + ' bytes, ' +
          str(size) + 'x' + str(size) + ' px)')


for s in SPRITES:
    gen(*s)
