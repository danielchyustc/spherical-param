#pragma once
#include <QString>
#include "QGLViewerWidget.h"
#include "MeshDefinition.h"
#include "MeshEdit/MeshEditor.h"
#include "MeshEdit/Parameterization/Parameterization.h"
#include "MeshEdit/Parameterization/ParaEnum.h"
#include "MeshEdit/Parameterization/Spherical/GaussMap.h"

class MeshViewerWidget : public QGLViewerWidget
{
	Q_OBJECT
public:
	MeshViewerWidget(QWidget* parent = 0);
	virtual ~MeshViewerWidget(void);
	bool LoadMesh(const std::string& filename);
	void Clear(void);
	void UpdateMesh(void);
	void UpdatePosition(void);
	bool SaveMesh(const std::string& filename);
	bool ScreenShot(void);
	void SetDrawBoundingBox(bool b);
	void SetDrawBoundary(bool b);
	void EnableLighting(bool b);
	void EnableDoubleSide(bool b);
	void ResetView(void);
	void ViewCenter(void);
	void CopyRotation(void);
	void LoadRotation(void);
	void InitEditor(void);
	void InitMinSurf(void);
	bool InitParam(void);
signals:
	void LoadMeshOKSignal(bool, QString);
public slots:
	void PrintMeshInfo(void);
	void EditorRun(void);
	void EditorReset(void);
	void ParamInit(void);
	void ParamReset(void);
	void ParamIter(void);
	void ParamITD(void);
	void ChangeParamViewMode(QString);
	void ChangeParamInitMethod(QString);
	void ChangeParamPlanarMethod(QString);
	void ChangeParamSphericalMethod(QString);
	void ChangeParamWeight(QString);
protected:
	virtual void DrawScene(void) override;
	void DrawSceneMesh(void);

private:
	void DrawPoints(void) const;
	void DrawWireframe(void) const;
	void DrawHiddenLines(void) const;
	void DrawFlatLines(void) const;
	void DrawFlat(void) const;
	void DrawSmooth(void) const;
	void DrawBoundingBox(void) const;
	void DrawBoundary(void) const;
protected:
	Mesh mesh;
	QString strMeshFileName;
	QString strMeshBaseName;
	QString strMeshPath;
	Mesh::Point ptMin;
	Mesh::Point ptMax;
	bool isEnableLighting;
	bool isTwoSideLighting;
	bool isDrawBoundingBox;
	bool isDrawBoundary;
	MeshEditor* editor;
	Parameterization* param;
};
