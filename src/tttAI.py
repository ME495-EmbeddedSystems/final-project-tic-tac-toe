#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import rospy

def printe(string):
    print(string, end='')

def print_array(gameboard):
    print()
    for i in range(len(gameboard)):
        for j in range(len(gameboard[i])):
            printe(gameboard[i][j])
            printe(", ")
        print()

def findXYOffset(tuple):
    homeX = -4.5
    homeY = 4.5
    baseX = -9
    baseY = 9
    xScale = 2-tuple[0]
    yScale = 2-tuple[1]
    xOffset = homeX + baseX * xScale
    yOffset = homeY + baseY * yScale

    unitScale = 1 #use this for changing units from cm
    return (xOffset*unitScale,yOffset*unitScale)

def findNextMove(gameboard, robot_player):
    nextMove = [-1, -1]
    rowSize = 3
    colSize = 3
    other_player = 1
    if robot_player == 1:
        other_player = 2

    #test for valid gameboard
    if len(gameboard) != rowSize:
        raise TypeError("Tic Tac Toe Board expected " + str(rowSize) + " rows, got " + str(len(gameboard)))
    for i in range(3):
        if len(gameboard[i]) != 3:
            raise TypeError("Tic Tac Toe Board expected " + str(colSize) + " columns, got " + str(len(gameboard[i])))
    ###

    #test to see if it is the robots turn
    currentMove = 0
    total_robot_moves = 0
    total_other_moves = 0
    for r in range(rowSize):
        for c in range(colSize):
            val = gameboard[r][c]
            if val == robot_player:
                total_robot_moves += 1
            if val == other_player:
                total_other_moves += 1
    currentMove = total_other_moves+total_robot_moves

    if(total_robot_moves > total_other_moves):
        print("Not the robots turn")
        return (-1,-1)
    ###

    #hard coded first move (go for center)
    if currentMove == 0:
        return (1,1)
    if currentMove == 1:
        if gameboard[1][1] == 0:
            return (1,1)
        return (0,0)
    ###

    weights = [[0,0,0],[0,0,0],[0,0,0]]

    for i in range(3):
        # get row weight
        pRobot = 0
        pOther = 0
        for k in range(3):
            cell = gameboard[i][k]
            if cell == robot_player:
                pRobot += 1
            if cell == other_player:
                pOther += 1
        w = pRobot - pOther
        if w > 1:
            w = 10
        if w < -1:
            w = 5

        # save row weight
        for k in range(3):
            if gameboard[i][k] != 0:
                weights[i][k] = -1
            else:
                weights[i][k] += abs(w)


        # get col weight
        pRobot = 0
        pOther = 0
        for k in range(3):
            cell = gameboard[k][i]
            if cell == robot_player:
                pRobot += 1
            if cell == other_player:
                pOther += 1
        w = pRobot - pOther
        if w > 1:
            w = 10
        if w < -1:
            w = 5

        # save col weight
        for k in range(3):
            if gameboard[k][i] != 0:
                weights[k][i] = -1
            else:
                weights[k][i] += abs(w)

        # end of loop

    # get left diagonal weight
    pRobot = 0
    pOther = 0
    for k in range(3):
        cell = gameboard[k][k]
        if cell == robot_player:
            pRobot += 1
        if cell == other_player:
            pOther += 1
    w = pRobot - pOther
    if w > 1:
        w = 10
    if w < -1:
        w = 5

    for k in range(3):
        if gameboard[k][k] != 0:
            weights[k][k] = -1
        else:
            weights[k][k] += abs(w)

    # get Right diagonal weight
    pRobot = 0
    pOther = 0
    for k in range(3):
        cell = gameboard[k][2-k]
        if cell == robot_player:
            pRobot += 1
        if cell == other_player:
            pOther += 1
    w = pRobot - pOther
    if w > 1:
        w = 10
    if w < -1:
        w = 5

    for k in range(3):
        if gameboard[k][2-k] != 0:
            weights[k][2-k] = -1
        else:
            weights[k][2-k] += abs(w)




    print_array(weights)

    maxWeight = -1
    coor = (-1,-1)
    for i in range(3):
        for k in range(3):
            test_weight = weights[i][k]
            if test_weight > maxWeight:
                print(test_weight)
                maxWeight = test_weight
                coor = (i,k)

    return coor

def convertCoor(tuple):
    return (-tuple[0]+1,-tuple[1]+1)

def convertShape2String(shapeInt):
    if shapeInt == -1:
        return "end"
    if shapeInt == 0:
        return "idle"
    if shapeInt == 1:
        return "cross"
    if shapeInt == 2:
        return "circle"
    return "error"

def rotateBoard(board):
    newBoard = np.copy(board)
    for i in range(3):
        for k in range(3):
            newBoard[2-i][2-k] = board[i][k]
    return newBoard

def Update(camera):

    camera.cameras.start_streaming(camera.argsCamera)

    rospy.sleep(5)
    camera.cameras.stop_streaming(camera.argsCamera)
    move = findNextMove(rotateBoard(camera.gamestate), 1)
    print_array(rotateBoard(camera.gamestate))
    print(move)
    if move[0] == -1:
        return (0,0),0
    if move[1] == -1:
        return (0,0),0
    return move,1

if __name__ == "__main__":
    rospy.init_node("sawyer")

    board = [[2,1,2],[2,1,1],[1,2,0]]
    bot = 1
    move = findNextMove(board,bot)
    print("Move: " + str(move))
    print(findXYOffset(move))
    rospy.spin()
