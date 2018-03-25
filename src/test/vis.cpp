#include <iostream>
#include <vector>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackball.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>

#include "../utils.h"


using namespace std;

vtkSmartPointer<vtkActor> drawPPFeature(Eigen::Vector3d& pt1, Eigen::Vector3d& nor1,Eigen::Vector3d& pt2,Eigen::Vector3d& nor2);

int main(int argc, char* argv[])
{
    vtkSmartPointer<vtkPoints> m_Points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices =vtkSmartPointer<vtkCellArray>::New();   //_存放细胞顶点，用于渲染（显示点云所必须的）
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkRenderer> ren1=vtkSmartPointer< vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> iren =vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> istyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();

    //_读进点云数据信息

    std::string filename("/home/sun/ClionProjects/ppf3Detector/data/mian_T-rex_high.ply");
    std::vector<Eigen::Vector3d> pts;
    std::vector<Eigen::Vector3d> nos;
    loadPLY(filename,pts,nos);


//    FILE*fp = NULL; fp=fopen("E:\\斯坦福兔子3000点.txt","r"); //2DpointDatas.txt
//    if(!fp)
//    {
//
//        printf("打开文件失败！！\n");
//        int m;
//        cin>>m;
//        exit(0);
//    }
    for (int j = 0; j < pts.size(); ++j) {
        m_Points->InsertPoint(j,pts[j].x(),pts[j].y(),pts[j].z());
        vertices->InsertNextCell(1);     //_加入细胞顶点信息----用于渲染点集
        vertices->InsertCellPoint(j);
    }

//    lineSource->SetPoint1(pts[0].x(),pts[0].y(),pts[0].z());
//    Eigen::Vector3d pt2 = pts[0] + nos[0];
//    lineSource->SetPoint2(pt2.x(),pt2.y(),pt2.z());
//
//    vtkSmartPointer<vtkPolyDataMapper> mapper =
//            vtkSmartPointer<vtkPolyDataMapper>::New();
//    mapper->SetInputConnection(lineSource->GetOutputPort());
//    vtkSmartPointer<vtkActor> actor =
//            vtkSmartPointer<vtkActor>::New();
//    actor->SetMapper(mapper);
//    actor->GetProperty()->SetLineWidth(2);
//    actor->GetProperty()->SetColor(1.0,0,0);


//    double x=0,y=0,z=0;
//    int i = 0;
//    while (!feof(fp))
//    {
//        fscanf(fp,"%lf %lf %lf",&x,&y,&z);
//        m_Points->InsertPoint(i,x,y,z);      //_加入点信息
//        vertices->InsertNextCell(1);     //_加入细胞顶点信息----用于渲染点集
//        vertices->InsertCellPoint(i);
//        i ++;
//    }
//    fclose(fp);

    //_创建待显示数据源

    polyData->SetPoints(m_Points);       //_设置点集
    polyData->SetVerts(vertices);        //_设置渲染顶点
    pointMapper->SetInput( polyData );

    pointActor->SetMapper( pointMapper );
    pointActor->GetProperty()->SetColor(0.0,0.1,1.0);
    pointActor->GetProperty()->SetAmbient(0.5);
    pointActor->GetProperty()->SetPointSize(2);
    //pointActor->GetProperty()->SetRepresentationToWireframe();
    //pointActor->GetProperty()->SetRepresentationToSurface();

    ren1->AddActor( pointActor );
//    ren1->AddActor( actor);
    ren1->AddActor(drawPPFeature(pts[0],nos[0],pts[12],nos[12]));
    ren1->SetBackground( 0, 0, 0);

    renWin->AddRenderer( ren1 );
    renWin->SetSize(800,800);

    iren->SetInteractorStyle(istyle);
    iren->SetRenderWindow(renWin);  //交互

    renWin->Render();
    iren->Start();
    return 0;
}

vtkSmartPointer<vtkActor> drawPPFeature(Eigen::Vector3d& pt1, Eigen::Vector3d& nor1,Eigen::Vector3d& pt2,Eigen::Vector3d& nor2)
{
//    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//
//    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//
//    Eigen::Vector3d pt1Vertex = pt1 + nor1;
//    Eigen::Vector3d pt2Vertex = pt2 + nor2;
//
//    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//    points->InsertNextPoint(pt1Vertex.x(),pt1Vertex.y(),pt1Vertex.z());
//    points->InsertNextPoint(pt1.x(),pt1.y(),pt1.z());
//    points->InsertNextPoint(pt2.x(),pt2.y(),pt2.z());
//    points->InsertNextPoint(pt2Vertex.x(),pt2Vertex.y(),pt2Vertex.z());
//
//    vtkIdType connectivity1[2];
//    connectivity1[0] = 0;
//    connectivity1[1] = 1;
//    connectivity1[2] = 2;
//    connectivity1[3] = 3;
//
////    vtkIdType connectivity2[2];
////    connectivity2[0] = 1;
////    connectivity2[1] = 2;
////
////    vtkIdType connectivity3[2];
////    connectivity3[0] = 2;
////    connectivity3[1] = 3;
//    polydata->Allocate();
//    polydata->SetPoints(points);
//    polydata->InsertNextCell(VTK_LINE,4,connectivity1);
//
////    polydata->InsertNextCell(VTK_LINE,2,connectivity2);
////    polydata->InsertNextCell(VTK_LINE,2,connectivity3);
//
//
//
//
//    mapper->SetInput(polydata);
//    actor->SetMapper(mapper);
//    actor->GetProperty()->SetLineWidth(2);
//    actor->GetProperty()->SetColor(1.0,0,0);

    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    Eigen::Vector3d pt1Vertex = pt1 + nor1;
    Eigen::Vector3d pt2Vertex = pt2 + nor2;

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(pt1Vertex.x(),pt1Vertex.y(),pt1Vertex.z());
    points->InsertNextPoint(pt1.x(),pt1.y(),pt1.z());
    points->InsertNextPoint(pt2.x(),pt2.y(),pt2.z());
    points->InsertNextPoint(pt2Vertex.x(),pt2Vertex.y(),pt2Vertex.z());

    linesPolyData->SetPoints(points);

    vtkSmartPointer<vtkLine> line0 =
            vtkSmartPointer<vtkLine>::New();
    line0->GetPointIds()->SetId(0, 0); // the second 0 is the index of the Origin in linesPolyData's points
    line0->GetPointIds()->SetId(1, 1); // the second 1 is the index of P0 in linesPolyData's points

    vtkSmartPointer<vtkLine> line1 =
            vtkSmartPointer<vtkLine>::New();
    line1->GetPointIds()->SetId(0, 1); // the second 0 is the index of the Origin in linesPolyData's points
    line1->GetPointIds()->SetId(1, 2); // 2 is the index of P1 in linesPoly

    vtkSmartPointer<vtkLine> line2 =
            vtkSmartPointer<vtkLine>::New();
    line2->GetPointIds()->SetId(0, 2); // the second 0 is the index of the Origin in linesPolyData's points
    line2->GetPointIds()->SetId(1, 3); // 2 is the index of P1 in linesPoly

    vtkSmartPointer<vtkCellArray> lines =
            vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(line0);
    lines->InsertNextCell(line1);
    lines->InsertNextCell(line2);

    linesPolyData->SetLines(lines);

    unsigned char red[3] = { 255, 0, 0 };
    unsigned char green[3] = { 0, 255, 0 };
    unsigned char white[3] = { 255, 255, 255 };

    vtkSmartPointer<vtkUnsignedCharArray> colors =
            vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->InsertNextTupleValue(red);
    colors->InsertNextTupleValue(green);
    colors->InsertNextTupleValue(white);

    linesPolyData->GetCellData()->SetScalars(colors);

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInput(linesPolyData);

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetColor(1.0,0,0);

    return actor;
}
