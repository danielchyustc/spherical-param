#pragma once

#include <QWidget>
#include <QtGui>
#include <QtWidgets>

class MeshParamWidget : public QWidget
{
	Q_OBJECT

public:
	MeshParamWidget(QWidget* parent = 0);
	~MeshParamWidget(void);
	void AddEditorTab(const QString tab_name);
	void AddParamTab(const bool has_boundary);
	void Clear();
private:
	void CreateTabWidget(void);
	void CreateLayout(void);
private slots:
	void CloseTab(int);
signals:
	void PrintInfoSignal();
	void RunSignal();
	void ResetSignal();
	void ParamInitSignal();
	void ParamIterSignal();
	void ParamITDSignal();
	void ParamResetSignal();
	void ParamViewModeChangedSignal(QString);
	void ParamInitMethodChangedSignal(QString);
	void ParamPlanarMethodChangedSignal(QString);
	void ParamSphericalMethodChangedSignal(QString);
	void ParamWeightChangedSignal(QString);
private:
	QTabWidget* twParam;
	QWidget* wInfo;
	QWidget* wEditor;
	QScrollArea* saInfo;
	QScrollArea* saEditor;
	QScrollArea* saParam;
	QPushButton* pbPrintInfo;
	QPushButton* pbRun;
	QPushButton* pbReset;
};
