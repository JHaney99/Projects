import pygame
import os
import math
import numpy as np
from board import *
import copy
import random
import time

pygame.init()
os.environ['SDL_VIDEO_CENTERED'] = '1'
pygame.display.set_caption('Dots and Boxes AI')
screen = pygame.display.set_mode((WIDTH, HEIGHT))

# top, bottom, left, right
board = np.zeros((n - 1, m - 1, 5), dtype=object)
# -1 human, 1 ai
turn = -1


def draw_dots():
    for i in range(m):
        for j in range(n):
            pygame.draw.circle(screen, BLACK, (i * WIDTH // m + OFFSET, j * HEIGHT // n + OFFSET), 7)


def draw_line(i, j, direction, colour):
    x1 = i * WIDTH // m + OFFSET
    x2 = x1 + WIDTH // m
    y1 = j * HEIGHT // n + OFFSET
    y2 = y1 + HEIGHT // n

    if direction == DIRECTION['top']:
        pygame.draw.line(screen, colour, (x1, y1), (x2, y1), THICC)
    elif direction == DIRECTION['bottom']:
        pygame.draw.line(screen, colour, (x1, y2), (x2, y2), THICC)
    elif direction == DIRECTION['left']:
        pygame.draw.line(screen, colour, (x1, y1), (x1, y2), THICC)
    if direction == DIRECTION['right']:
        pygame.draw.line(screen, colour, (x2, y1), (x2, y2), THICC)


def draw_board():
    global board
    for i in range(m - 1):
        for j in range(n - 1):
            for index, line in enumerate(board[j, i, :4]):
                try:
                    # Check if the lines are drawn
                    if line[0] != 0:
                        draw_line(i, j, index, line[1])
                except TypeError:
                    # line = [drawn, colour]
                    board[j, i, index] = (0, 0)


def colour_box():
    for i in range(m - 1):
        for j in range(n - 1):
            # Check if the box is full
            full = board[j, i][4]
            if full:
                pygame.draw.rect(screen, COLOURS[full * 2],
                                 (i * WIDTH // m + OFFSET + THICC - 1, j * HEIGHT // n + OFFSET + THICC - 1,
                                  WIDTH // m - THICC,
                                  HEIGHT // n - THICC))


def draw_text(size, text, colour, x, y):
    font = pygame.font.SysFont('Comic Sans MS', size)
    text_surface = font.render(text, 1, colour)
    text_rect = text_surface.get_rect(center=(x, y))
    screen.blit(text_surface, text_rect)


def reset():
    global board, turn, playing, winner
    winner = WHITE
    board = np.zeros((n - 1, m - 1, 5), dtype=object)
    draw_board()
    playing = True
    SCORES[1] = 0
    SCORES[-1] = 0
    turn = -1


# Function created to create a list of all available moves at a given time. Allows AI to test all possible moves (branches)
def available_moves(board):
    available_moves_list = []
    for i in range(m - 1):
        for j in range(n - 1):
            if board[j, i][DIRECTION['top']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                available_moves_list.append((i, j, 'top'))
            if board[j, i][DIRECTION['bottom']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                available_moves_list.append((i, j, 'bottom'))
            if board[j, i][DIRECTION['left']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                available_moves_list.append((i, j, 'left'))
            if board[j, i][DIRECTION['right']][1] in [LIGHT_RED, LIGHT_BLUE, 0]:
                available_moves_list.append((i, j, 'right'))
    return available_moves_list


# Evaluation function to test what value a play will have
# Note, must be a TEMP score, as the function implimeneted will use the real score otherwise (affecting final scoring output)
def evaluate(board):
    check_full(board, 1, True)
    check_full(board, -1, True) 
    return TEMP_SCORES[1] - TEMP_SCORES[-1]


# Minimax function with alpha beta pruning
def minimax(board, depth, alpha, beta, maximizing_player):
    # Base case: if the game is over or the depth limit is reached
    if depth == 0 or check_board_full(board):  #If the depth has reached 0 it will evaluate the total benefit (or the board has no more room)
        return evaluate(board)
    

    # Max
    if maximizing_player:
        max_eval = float('-inf')
        for move in available_moves(board):   #Iterate over all possible plays on a board
            eval = minimax(board, depth - 1, alpha, beta, False) #Swap between max and min, decreasing depth as you go
            max_eval = max(max_eval, eval)   #Find new max
            alpha = max(alpha, eval)
            if alpha >= beta:   #Prune if necessary
                break
        return max_eval
    
    #Min
    else:
        min_eval = float('inf')
        for move in available_moves(board): #Same as max, iterate over all moves
            eval = minimax(board, depth - 1, alpha, beta, True)   #Descrease depth and alternate between max and min
            min_eval = min(min_eval, eval)   #Find new min value
            beta = min(beta, eval)
            if beta <= alpha:       #Prune in necessary
                break
        return min_eval


# Function to actually make AI make moves
def ai_move(board, depth):
    best_score = float('-inf')   #Best intial score must start at minus ininity
    best_move = None
    alpha = float('-inf')   #define alpha and beta because i dont want to keep typing it
    beta = float('inf')
    for move in available_moves(board):
        i, j, direction = move[0], move[1], move[2]   #Takes the available move from the list, seperates into 3 variables so place line can use it
        new_board = copy.deepcopy(board)    #Copy the board for testing because I dont want to affect the real one
        place_line(new_board, (i * WIDTH // m + OFFSET, j * HEIGHT // n + OFFSET), turn, direction)  #Test placing the line
        score = minimax(new_board, depth, alpha, beta, True)  #Find the score that that play will result in (we are maximuzing the AI score)
        if score > best_score:   #See if that score is better than previous and store the move
            best_score = score
            best_move = move
    if best_move is not None:  #This should never occur, this was more used for testing/failsafe
        i, j, direction = best_move  #Convert that best move into a useable format for place line
        return place_line(board, (i * WIDTH // m + OFFSET, j * HEIGHT // n + OFFSET), turn, direction)


# Convert board
draw_board()
playing = True
winner = WHITE

while True:
    screen.fill(winner)
    mouse_pos = pygame.mouse.get_pos()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                quit()
        if turn == 1: #AI's Turn
            if len(available_moves(board)) == 1: #Check to see if there is only one move remaining, if there is it just uses that
                   i, j, direction = available_moves(board)[0]
                   if place_line(board, (i * WIDTH // m + OFFSET, j * HEIGHT // n + OFFSET), turn, direction) and not check_full(board, turn, False):
                       turn *= -1
            if ai_move(board, 3) and not check_full(board, turn, False):
                turn *= -1
        else:
            if event.type == pygame.MOUSEBUTTONDOWN:
                assert math.fabs(turn) == 1
                if event.button == pygame.BUTTON_LEFT:
                    if place_line(board, mouse_pos, turn) and not check_full(board, turn, False):
                        turn *= -1
                if event.button == pygame.BUTTON_RIGHT:
                    reset()

    if playing:
        place_line(board, mouse_pos, turn * 2, None, True) #This is what gives the animation when you hover some dots (has nothing to due with actual placement)
        draw_board()   #Appears to be what allows you to actually place stuff
        colour_box()  #This is what colours the boxes when it is full
        draw_dots()   #This is what places the dots, however lines can be placed without out
        draw_text(40, f'{SCORES[1]}', RED, WIDTH // 1.5, HEIGHT - 40)
        draw_text(40, f'{SCORES[-1]}', BLUE, WIDTH // 3, HEIGHT - 40)
    
    if check_board_full(board):
        playing = False
        if SCORES[-1] > SCORES[1]:
            winner = COLOURS[-1]
        elif SCORES[-1] == SCORES[1]:
            winner = WHITE
        else:
            winner = COLOURS[1]
            if SCORES[1] > SCORES[-1]:
                draw_text(40, f'RED WON {SCORES[-1]}', BLUE, WIDTH // 3, HEIGHT - 40)
                time.sleep(5)
            elif SCORES[1] < SCORES[-1]:
                draw_text(40, f'BLUE WON {SCORES[1]}', RED, WIDTH // 3, HEIGHT - 40)
                time.sleep(5)

    pygame.display.flip()