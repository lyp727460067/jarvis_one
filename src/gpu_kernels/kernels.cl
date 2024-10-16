__kernel void fast_pyra_down(__global const uchar* input, __global uchar* output,
                             const int width) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int x_row = x + x;
    int y_row = y + y;
    int width2 = width + width;
    int index = y_row * width2 + x_row;

    int sum = (int)(input[index]) + (int)(input[index+1]) +
              (int)(input[index + width2]) + (int)(input[index + width2 +1]);
    sum = sum / 4;

    output[y * width + x] = (uchar)(sum);
}

__kernel void calc_scharr_deriv(__global const uchar* input, __global short* output,
                                const int width, const int height) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    
    int row1 = y * width;
    int row0 = (y > 0) ? (row1 - width) : (row1 + width);
    int row2 = (y < height - 1) ? (row1 + width) : (row1 - width);

    int col0 = (x > 0) ? (x - 1) : 1;
    int col2 = (x < width - 1) ? (x + 1) : (width - 2);

    int deriv_r = (input[row0 + col2] - input[row0 + col0] + input[row2 + col2]
                 - input[row2 + col0]) * 3 + (input[row1 + col2] - input[row1 + col0])
                 * 10;
    int deriv_c = (input[row2 + col2] - input[row0 + col2] + input[row2 + col0]
                 - input[row0 + col0]) * 3 + (input[row2 + x] - input[row0 + x]) * 10;

    output[row1 + row1 + x + x] = (short)deriv_r;
    output[row1 + row1 + x + x + 1] = (short)deriv_c;
}

__kernel void fast_pyra_down_with_border(__global const uchar* input, __global uchar* output,
                                         const int width ,const int height, const int winSize) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int winSize2 = winSize + winSize;
    int onerow = width + winSize2;
    int onerow2 = width + width + winSize2;

    int col = (x < winSize) ? (winSize2 - x) : ((x >= width + winSize) ? (onerow2 - x) : x);
    int row = (y < winSize) ? (winSize2 - y) : ((y >= height + winSize) ? 
              (height + height + winSize2 - y) : y);

    int index = (row + row - winSize) * onerow2 + col + col - winSize;

    int sum = (int)(input[index]) + (int)(input[index+1]) +
              (int)(input[index + onerow2]) + (int)(input[index + onerow2 +1]);
    sum = sum / 4;

    output[y * onerow + x] = (uchar)(sum);
}

__kernel void calc_scharr_deriv_with_border(__global const uchar* input,
                                            __global short* output,
                                            const int width, const int height,
                                            const int winSize) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int winSize2 = winSize + winSize;
    int onerow = width + winSize2;
    int onecol = height + winSize2;

    int col = (x < winSize) ? (winSize2 - x) : ((x >= width + winSize) ?
              (col = onerow + width - x) : x);
    int row = (y < winSize) ? (winSize2 - y) : ((y >= height + winSize) ?
              (row = onecol + height - y) : y);
    
    int row1 = row * onerow;
    int row0 = (row > winSize) ? (row1 - onerow) : (row1 + onerow);
    int row2 = (row < height + winSize - 1) ? (row1 + onerow) : (row1 - onerow);

    int col0 = (col > winSize) ? (col - 1) : 1;
    int col2 = (col < width + winSize - 1) ? (col + 1) : (width + winSize - 2);

    int deriv_r = (input[row0 + col2] - input[row0 + col0] + input[row2 + col2]
                 - input[row2 + col0]) * 3 + (input[row1 + col2] - input[row1 + col0])
                 * 10;
    int deriv_c = (input[row2 + col2] - input[row0 + col2] + input[row2 + col0]
                 - input[row0 + col0]) * 3 + (input[row2 + col] - input[row0 + col]) * 10;

    int rowsid = y * onerow;
    output[rowsid + rowsid + x + x] = (short)deriv_r;
    output[rowsid + rowsid + x + x + 1] = (short)deriv_c;    
}