from conf import *


def check_board_full(board):
    return 0 not in board[:, :, 4]

# Function to place lines. It was modified so that AI could make moves (hence direction=None) Player does not need a direction, however, AI need to be able to define what
# Direction the line is going to be placed
def place_line(board, pos, turn, direction=None, hover=False):
    x = pos[0]
    y = pos[1]
    # Loop through all the boxes
    if hover:
        for i in range(m - 1):
            for j in range(n - 1):
                for index, line in enumerate(board[j, i][:4]):
                    if line[0] == 1 and line[1] in [LIGHT_RED, LIGHT_BLUE]:
                        board[j, i][index] = (0, 0)
    for i in range(m - 1):
        for j in range(n - 1):

            top = j * HEIGHT // n + OFFSET
            bottom = (j + 1) * HEIGHT // n + OFFSET
            left = i * WIDTH // m + OFFSET
            right = (i + 1) * WIDTH // m + OFFSET
            # Check if the mouse point is in the current box
            if top <= y < bottom and left <= x < right:
                if direction is None:
                    # Find which edge the mouse is closest to
                    top_d = y - top
                    bottom_d = bottom - y
                    left_d = x - left
                    right_d = right - x
                    closest = min(top_d, bottom_d, left_d, right_d)
                    # Find the closest and if is not filled
                    if top_d == closest and board[j, i][DIRECTION['top']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['top']] = (1, COLOURS[turn])
                        # Check if there is a box above and then add a bottom edge
                        if j != 0:
                            board[j - 1, i][DIRECTION['bottom']] = (1, COLOURS[turn])
                        return True
                    elif bottom_d == closest and board[j, i][DIRECTION['bottom']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['bottom']] = (1, COLOURS[turn])
                        # Check if there is a box below and then add a top edge
                        if j != n - 2:
                            board[j + 1, i][DIRECTION['top']] = (1, COLOURS[turn])
                        return True
                    elif left_d == closest and board[j, i][DIRECTION['left']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['left']] = (1, COLOURS[turn])
                        # Check if there is a box to the left and then add a right edge
                        if i != 0:
                            board[j, i - 1][DIRECTION['right']] = (1, COLOURS[turn])
                        return True

                    elif right_d == closest and board[j, i][DIRECTION['right']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['right']] = (1, COLOURS[turn])
                        # Check if there is a box to the right and then add a left edge
                        if i != m - 2:
                            board[j, i + 1][DIRECTION['left']] = (1, COLOURS[turn])
                        return True
                else:  #This is the function that allows for AI to make moves based on 'coordinates'
                    # Place the line according to the given direction
                    if direction == 'top' and board[j, i][DIRECTION['top']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['top']] = (1, COLOURS[turn])
                        if j != 0:
                            board[j - 1, i][DIRECTION['bottom']] = (1, COLOURS[turn])
                        return True
                    elif direction == 'bottom' and board[j, i][DIRECTION['bottom']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['bottom']] = (1, COLOURS[turn])
                        if j != n - 2:
                            board[j + 1, i][DIRECTION['top']] = (1, COLOURS[turn])
                        return True
                    elif direction == 'left' and board[j, i][DIRECTION['left']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['left']] = (1, COLOURS[turn])
                        if i != 0:
                            board[j, i - 1][DIRECTION['right']] = (1, COLOURS[turn])
                        return True
                    elif direction == 'right' and board[j, i][DIRECTION['right']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                        board[j, i][DIRECTION['right']] = (1, COLOURS[turn])
                        if i != m - 2:
                            board[j, i + 1][DIRECTION['left']] = (1, COLOURS[turn])
                        return True



# This checks if a box is full to give a point. Modified so that the real score wont be affected when AI is testing
def check_full(board, turn, AI):
    if AI == False:
        # To handle double boxes
        full = False
        for i in range(m - 1):
            for j in range(n - 1):
                if board[j, i][4] != 0:
                    continue
                # Check if all sides are filled
                if list(zip(*board[j, i][:4]))[0] == (1, 1, 1, 1):
                    board[j, i][4] = turn
                    SCORES[turn] += 1
                    full = True
        return full
    if AI == True:
        # To handle double boxes
        full = False
        for i in range(m - 1):
            for j in range(n - 1):
                if board[j, i][4] != 0:
                    continue
                # Check if all sides are filled
                if list(zip(*board[j, i][:4]))[0] == (1, 1, 1, 1):
                    board[j, i][4] = turn
                    TEMP_SCORES[turn] += 1
                    full = True
        return full