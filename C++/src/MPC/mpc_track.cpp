#include "mpc_track.hpp"

namespace mpcc
{

MpcTrack::MpcTrack(const Config &config): d_centerLine(config), d_outerBorder(config), d_innerBorder(config)
{
}

void MpcTrack::generate(const Eigen::VectorXd& x, const Eigen::VectorXd& y, 
						const Eigen::VectorXd& xOuter, const Eigen::VectorXd& yOuter, 
						const Eigen::VectorXd& xInner, const Eigen::VectorXd& yInner)
{
	d_centerLine.gen2DSpline(x, y);
    d_outerBorder.setPath(xOuter, yOuter);
    d_innerBorder.setPath(xInner, yInner);
    
    calculateBordersInterpolations();
}

void MpcTrack::calculateBordersInterpolations(){
    // Build perpendicular-offset border interpolations w.r.t. centerline normals
    auto centerLinePath = d_centerLine.getPath();
    int nPts = centerLinePath.n_points;

    Eigen::VectorXd outerPerpX(nPts);
    Eigen::VectorXd outerPerpY(nPts);
    Eigen::VectorXd innerPerpX(nPts);
    Eigen::VectorXd innerPerpY(nPts);

    outerPerpX.setZero();
    outerPerpY.setZero();
    innerPerpX.setZero();
    innerPerpY.setZero();

    // Use resampled border paths for nearest-neighbor search
    const PathData& outerBorderPath = d_outerBorder.getPath();
    const PathData& innerBorderPath = d_innerBorder.getPath();

    const Eigen::VectorXd& outerX = outerBorderPath.X;
    const Eigen::VectorXd& outerY = outerBorderPath.Y;
    const Eigen::VectorXd& innerX = innerBorderPath.X;
    const Eigen::VectorXd& innerY = innerBorderPath.Y;

    for (int i = 0; i < nPts; i++)
    {
      double s = centerLinePath.s(i);
      Eigen::Vector2d centerPos = d_centerLine.getPosition(s);
      Eigen::Vector2d tangent = d_centerLine.getDerivative(s);

      double cx = centerPos(0);
      double cy = centerPos(1);
      double tx = tangent(0);
      double ty = tangent(1);
      
      double tnorm = std::hypot(tx,ty);
      if(tnorm > 0)
      {
        tx = tx/tnorm; 
        ty = ty/tnorm;
      }

      double nx = -ty; 
      double ny = tx; // Левая нормаль

      /* --- РАСЧЕТ ДЛЯ ВНЕШНЕЙ ГРАНИЦЫ (Вдоль +N) */
      auto [xO, yO] = findRayBorderIntersection(cx, cy, nx, ny, outerX, outerY);
      outerPerpX(i) = xO;
      outerPerpY(i) = yO;

      /* --- РАСЧЕТ ДЛЯ ВНУТРЕННЕЙ ГРАНИЦЫ (Вдоль -N) */
      auto [xI, yI] = findRayBorderIntersection(cx, cy, -nx, -ny, innerX, innerY);
      innerPerpX(i) = xI;
      innerPerpY(i) = yI;
    }

    d_outerBorder.genBorderInterpolation(outerPerpX, outerPerpY, centerLinePath.s);
    d_innerBorder.genBorderInterpolation(innerPerpX, innerPerpY, centerLinePath.s);
  }

  std::pair<double, double> MpcTrack::findRayBorderIntersection(double cx, double cy, double nx, double ny, const Eigen::VectorXd& bx, const Eigen::VectorXd& by){        
    double bestX = cx; 
    double bestY = cy;
    double minT = 1e9; // Ищем минимальный положительный шаг вдоль луча
    
    double nSegs = bx.rows() - 1;
    
    for(int j = 0; j < nSegs; j++){
      // Вершины текущего сегмента границы
      double x1 = bx(j);   
      double y1 = by(j);
      double x2 = bx(j+1); 
      double y2 = by(j+1);
      
      // Вектор сегмента границы
      double dx = x2 - x1;
      double dy = y2 - y1;
      
      // Знаменатель (определитель матрицы системы)
      double det = nx * dy - ny * dx;
      
      // Если det == 0, луч и сегмент параллельны
      if(std::abs(det) < 1e-9)
      {
        continue;
      }

      // Решение системы линейных уравнений по правилу Крамера
      // t - расстояние вдоль луча нормали
      // u - положение точки на отрезке границы (от 0 до 1)
      double t = ((x1 - cx) * dy - (y1 - cy) * dx) / det;
      double u = ((x1 - cx) * ny - (y1 - cy) * nx) / det;
      
      // Проверяем, что пересечение впереди по лучу и попадает на отрезок
      if(t >= 0 && u >= 0 && u <= 1){
        if(t < minT)
        {
          minT = t;
          bestX = cx + t * nx;
          bestY = cy + t * ny;
        }
      }
    }
    return {bestX, bestY};
  }
  } // namespace mpcc

