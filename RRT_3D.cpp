//g++ cpp_RRT.cpp -o t -I/usr/include/python3.8 -lpython3.8
#include <random>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <limits>

#define PLOT

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

double MAX_ITERATION = 80000;
double MAX_DISTANCE = 0.8;
double GOAL;

//obstacles for RRT
std::vector<double> obsXmin{3};
std::vector<double> obsYmin{3};
std::vector<double> obsZmin{3};
std::vector<double> obsXmax{7};
std::vector<double> obsYmax{10};
std::vector<double> obsZmax{10};

//------------------------------------------------------------------------------

struct Node
{

    double posX;
    double posY;
    double posZ;
    Node *prev;
    Node *next;
};

//------------------------------------------------------------------------------

class RRT
{

private:
    Node *start;
    Node *goal;
    std::vector<Node *> rrtNodes;

public:
    RRT(double startX, double startY, double startZ, double goalX, double goalY, double goalZ)
    {

        Node *node = new Node;
        this->start = node;
        start->posX = startX;
        start->posY = startY;
        start->posZ = startZ;
        start->prev = nullptr;
        rrtNodes.push_back(node);

        node = new Node;
        this->goal = node;
        goal->posX = goalX;
        goal->posY = goalY;
        goal->posZ = goalZ;
    }

    //------------------------------------------------------------------------------

#ifdef PLOT

    void printGoalPath(std::tuple<std::vector<double>, std::vector<double>> path)
    {
        std::vector<double> xX = std::get<0>(path);
        std::vector<double> yY = std::get<1>(path);

        //obstacles for visualisation
        std::vector<int> obsX1{3, 7, 7, 3, 3};
        std::vector<int> obsY1{3, 3, 10, 10, 3};

        std::vector<int> obsX2{11, 18, 18, 11, 11};
        std::vector<int> obsY2{6, 6, 17, 17, 6};

        plt::figure_size(600, 600);
        plt::plot(obsX1, obsY1, "darkblue");
        plt::plot(obsX2, obsY2, "darkblue");
        plt::plot(xX, yY, "r--");
        plt::xlabel("xx");
        plt::ylabel("yy");
        plt::xlim(-1, 21);
        plt::ylim(-1, 21);

        plt::show();
    }

    //------------------------------------------------------------------------------

    void plotRRT()
    {

        std::vector<double> xX;
        std::vector<double> yY;

        for (auto &ii : rrtNodes)
        {

            xX.push_back(ii->posX);
            yY.push_back(ii->posY);
        }

        plt::figure_size(600, 600);
        plt::plot(xX, yY);
        plt::xlabel("xx");
        plt::ylabel("yy");
        plt::xlim(0, 20);
        plt::ylim(0, 20);

        plt::show();
    }
#endif

    //------------------------------------------------------------------------------
    void plot3dRRT(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> path)
    {
        std::cout << __FUNCTION__ << "\n";

        std::vector<int> obsX1{3, 7, 7, 3, 3};
        std::vector<int> obsY1{3, 3, 10, 10, 3};

        plt::plot(obsX1, obsY1, "darkblue");
        plt::title("obstacle XY space");
        plt::xlim(-1, 21);
        plt::ylim(-1, 21);
        plt::xlabel("xx");
        plt::ylabel("yy");

        std::vector<double> xX = std::get<0>(path);
        std::vector<double> yY = std::get<1>(path);
        std::vector<double> zZ = std::get<2>(path);

        plt::plot3(xX, yY, zZ);
        plt::xlabel("x");
        plt::ylabel("y");
        plt::set_zlabel("z");
        plt::show();
    }

    //------------------------------------------------------------------------------

    bool checkObstacles(double x1, double y1, double z1, double x2, double y2, double z2, std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> obs)
    {
        bool decision;

        std::vector<double> minX = std::get<0>(obs);
        std::vector<double> minY = std::get<1>(obs);
        std::vector<double> minZ = std::get<2>(obs);
        std::vector<double> maxX = std::get<3>(obs);
        std::vector<double> maxY = std::get<4>(obs);
        std::vector<double> maxZ = std::get<5>(obs);

        for (int i = 0; i < minX.size(); i++)
        {

            // Completely outside.
            if ((x1 <= minX[i] && x2 <= minX[i]) || (y1 <= minY[i] && y2 <= minY[i]) || (x1 >= maxX[i] && x2 >= maxX[i]) || (y1 >= maxY[i] && y2 >= maxY[i]))
                decision = false;
        }

        for (int i = 0; i < minX.size(); i++)
        {

            double m = (y2 - y1) / (x2 - x1);

            double y = m * (minX[i] - x1) + y1;
            if (y > minY[i] && y < maxY[i])
                decision = true;

            y = m * (maxX[i] - x1) + y1;
            if (y > minY[i] && y < maxY[i])
                decision = true;

            double x = (minY[i] - y1) / m + x1;
            if (x > minX[i] && x < maxX[i])
                decision = true;

            x = (maxY[i] - y1) / m + x1;
            if (x > minX[i] && x < maxX[i])
                decision = true;
        }

        for (int i = 0; i < minX.size(); i++)
        {

            if ((minZ[i] > z1 && minZ[i] > z2) || (maxZ[i] < z1 && maxZ[i] < z2))
            {
                decision = true;
            }
        }

        return decision;
    }

    //------------------------------------------------------------------------------

    std::tuple<double, double, double> compute3DLineCorr(std::vector<double> p0, std::vector<double> p1)
    {
        double dt = 0.01;

        double xX;
        double yY;
        double zZ;

        xX = (p1[0] - p0[0]) * MAX_DISTANCE + p0[0];
        yY = (p1[1] - p0[1]) * MAX_DISTANCE + p0[1];
        zZ = (p1[2] - p0[2]) * MAX_DISTANCE + p0[2];

        return std::make_tuple(xX, yY, zZ);
    }

    //------------------------------------------------------------------------------

    Node *checkNearestNode(Node *new_node)
    {
        std::vector<double> tempX;
        std::vector<double> tempY;
        std::vector<double> tempZ;

        Node *near_node = new Node;
        double minDistance = std::numeric_limits<double>::max();
        double corrX = 0.0;
        double corrY = 0.0;
        double corrZ = 0.0;
        bool check_obstacle;

        for (auto &ii : rrtNodes)
        {

            double distance = std::sqrt(std::pow((new_node->posX - ii->posX), 2) + std::pow((new_node->posY - ii->posY), 2) + std::pow((new_node->posZ - ii->posZ), 2));
            if (distance < minDistance)
            {

                minDistance = distance;
                near_node = ii;
            }
        }

        if (minDistance > MAX_DISTANCE)
        {

            std::vector<double> P1 = {new_node->posX, new_node->posY, new_node->posZ};
            std::vector<double> P0 = {near_node->posX, near_node->posY, near_node->posZ};
            std::tuple<double, double, double> corrXYZ = compute3DLineCorr(P0, P1);

            corrX = std::get<0>(corrXYZ);
            corrY = std::get<1>(corrXYZ);
            corrZ = std::get<2>(corrXYZ);
        }

        if (minDistance <= MAX_DISTANCE)
        {

            corrX = new_node->posX;
            corrY = new_node->posY;
            corrZ = new_node->posZ;
        }
        if (rrtNodes.size() > 0)
        {

            check_obstacle = checkObstacles(near_node->posX, near_node->posY, near_node->posZ, corrX, corrY, corrZ, std::make_tuple(obsXmin, obsYmin, obsZmin, obsXmax, obsYmax, obsZmax));
        }

        new_node->posX = corrX;
        new_node->posY = corrY;
        new_node->posZ = corrZ;

        near_node->next = new_node;
        new_node->prev = near_node;

        if (rrtNodes.size() == 0)
        {

            new_node->prev = start;
        }

        if (check_obstacle == 0)
        {
            rrtNodes.push_back(new_node);
        }

        if (((double)new_node->posX == (double)this->goal->posX) && ((double)new_node->posY == (double)this->goal->posY) && ((double)new_node->posZ == (double)this->goal->posZ))
        {

            std::cout << "The GOAL achive && GOLD path is ..." << std::endl;

            GOAL = 1;

            while (new_node->prev != nullptr)

            {
                std::cout << " :X: " << new_node->posX << " :Y: " << new_node->posY << " :Z: " << new_node->posZ << "\n";
                new_node = new_node->prev;
                tempX.push_back(new_node->posX);
                tempY.push_back(new_node->posY);
                tempZ.push_back(new_node->posZ);
            }

            plot3dRRT(std::make_tuple(tempX, tempY, tempZ));
        }

        return new_node;
    }

    //------------------------------------------------------------------------------

    double lookForPath()
    {

        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist100(0, 20); // distribution in range [1, 6]

        for (double i = 0; i < MAX_ITERATION; i++)
        {
            Node *random_node = new Node;
            Node *last_node = new Node;

            double randX = dist100(rng);
            double randY = dist100(rng);
            double randZ = dist100(rng);
            random_node->posX = randX;
            random_node->posY = randY;
            random_node->posZ = randZ;

            last_node = checkNearestNode(random_node);

            if (GOAL == 1)
            {
                goal->prev = last_node;
                std::cout << last_node->posX << " :: " << last_node->posY << " :: " << last_node->posZ << "\n";

                return -1;
            }
        }
        return 1;
    }

    //------------------------------------------------------------------------------

    void printRRT(RRT &rrt)
    {

        Node *node = rrt.start;
        std::cout << node->posX << " : " << node->posY << std::endl;
        node = rrt.goal;
        std::cout << node->posX << " : " << node->posY << std::endl;
    }
};

//------------------------------------------------------------------------------

void printObsXY()
{

    //obstacles for visualisation
    std::vector<int> obsX1{3, 7, 7, 3, 3};
    std::vector<int> obsY1{3, 3, 10, 10, 3};

    std::vector<int> obsX2{11, 18, 18, 11, 11};
    std::vector<int> obsY2{6, 6, 17, 17, 6};

    plt::figure_size(600, 600);
    plt::plot(obsX1, obsY1, "darkblue");
    plt::plot(obsX2, obsY2, "darkblue");
    plt::xlabel("xx");
    plt::ylabel("yy");
    plt::xlim(-1, 21);
    plt::ylim(-1, 21);

    plt::show();
}

//------------------------------------------------------------------------------

int main()
{

    RRT rrt(2, 5, 2, 10, 10, 10); //fig 2

    //RRT rrt(2, 2, 2, 14, 10, 8); // fig 1
    //rrt.printRRT(rrt);

    double checkpath = rrt.lookForPath();
}
