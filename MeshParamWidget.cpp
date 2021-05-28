#include "MeshParamWidget.h"
#include <iostream>

using namespace std;

MeshParamWidget::MeshParamWidget(QWidget* parent)
	: QWidget(parent)
{
	CreateTabWidget();
	CreateLayout();
}

MeshParamWidget::~MeshParamWidget()
{
}

void MeshParamWidget::CreateTabWidget(void)
{
	pbPrintInfo = new QPushButton(tr("Print Mesh Info"));
	connect(pbPrintInfo, SIGNAL(clicked()), SIGNAL(PrintInfoSignal()));

	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(pbPrintInfo);
	layout->addStretch();
	wInfo = new QWidget();
	wInfo->setLayout(layout);
	saInfo = new QScrollArea();
	saInfo->setFocusPolicy(Qt::NoFocus);
	saInfo->setFrameStyle(QFrame::NoFrame);
	saInfo->setWidget(wInfo);
	saInfo->setWidgetResizable(true);
}

void MeshParamWidget::CreateLayout(void)
{
	twParam = new QTabWidget();
	twParam->addTab(saInfo, "Info");
	twParam->setTabsClosable(true);
	connect(twParam, SIGNAL(tabCloseRequested(int)), this, SLOT(CloseTab(int)));
	QGridLayout* layout = new QGridLayout();
	layout->addWidget(twParam, 0, 0, 1, 1);
	this->setLayout(layout);
}

void MeshParamWidget::Clear()
{
	twParam->removeTab(twParam->indexOf(saEditor));
	twParam->removeTab(twParam->indexOf(saParam));
}

void MeshParamWidget::CloseTab(int index)
{
	if (index == -1) return;
	QWidget* tabItem = twParam->widget(index);
	twParam->removeTab(index);
}

void MeshParamWidget::AddEditorTab(const QString tab_name)
{
	pbRun = new QPushButton(tr("Run"));
	connect(pbRun, SIGNAL(clicked()), SIGNAL(RunSignal()));
	pbReset = new QPushButton(tr("Reset"));
	connect(pbReset, SIGNAL(clicked()), SIGNAL(ResetSignal()));

	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(pbRun);
	layout->addWidget(pbReset);
	layout->addStretch();
	wEditor = new QWidget();
	wEditor->setLayout(layout);
	saEditor = new QScrollArea();
	saEditor->setFrameStyle(QFrame::NoFrame);
	saEditor->setWidget(wEditor);
	saEditor->setWidgetResizable(true);
	twParam->setCurrentIndex(twParam->addTab(saEditor, tab_name));
	saEditor->setFocus();
}


void MeshParamWidget::AddParamTab(const bool has_boundary)
{
	QLabel* lParamViewMode = new QLabel("View Mode:");
	QComboBox* cbParamPlanarViewMode = new QComboBox();
	cbParamPlanarViewMode->addItem("Mesh");
	cbParamPlanarViewMode->addItem("Plane");
	cbParamPlanarViewMode->setCurrentText("Plane");
	connect(cbParamPlanarViewMode, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamViewModeChangedSignal(QString)));

	QComboBox* cbParamSphericalViewMode = new QComboBox();
	cbParamSphericalViewMode->addItem("Mesh");
	cbParamSphericalViewMode->addItem("Sphere");
	cbParamSphericalViewMode->setCurrentText("Sphere");
	connect(cbParamSphericalViewMode, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamViewModeChangedSignal(QString)));

	QLabel* lParamInitMethod = new QLabel("Init Method:");
	QComboBox* cbParamSphericalInitMethod = new QComboBox();
	cbParamSphericalInitMethod->addItem("Mesh Voronoi");
	cbParamSphericalInitMethod->addItem("Hier Cluster");
	cbParamSphericalInitMethod->setCurrentText("Mesh Voronoi");
	connect(cbParamSphericalInitMethod, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamInitMethodChangedSignal(QString)));

	QLabel* lParamSolver = new QLabel("Solver:");
	QComboBox* cbParamPlanarSolver = new QComboBox();
	cbParamPlanarSolver->addItem("Tutte");
	cbParamPlanarSolver->addItem("Progressive");
	cbParamPlanarSolver->setCurrentText("Progressive");
	connect(cbParamPlanarSolver, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamPlanarMethodChangedSignal(QString)));

	QComboBox* cbParamSphericalSolver = new QComboBox();
	cbParamSphericalSolver->addItem("GaussMap");
	cbParamSphericalSolver->addItem("SGD");
	cbParamSphericalSolver->addItem("SLIM");
	cbParamSphericalSolver->addItem("AKVF");
	cbParamSphericalSolver->addItem("APN");
	cbParamSphericalSolver->addItem("Hybrid Solver");
	cbParamSphericalSolver->setCurrentText("Hybrid Solver");
	connect(cbParamSphericalSolver, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamSphericalMethodChangedSignal(QString)));

	QLabel* lParamWeight = new QLabel("Weight:");
	QComboBox* cbParamWeight = new QComboBox();
	cbParamWeight->addItem("Uniform");
	cbParamWeight->addItem("Cotangent");
	cbParamWeight->setCurrentText("Cotangent");
	connect(cbParamWeight, SIGNAL(currentTextChanged(QString)), SIGNAL(ParamWeightChangedSignal(QString)));

	QLabel* lParamOperations = new QLabel("Operations:");
	QPushButton* pbParamRun = new QPushButton(tr("Run"));
	connect(pbParamRun, SIGNAL(clicked()), SIGNAL(ParamInitSignal()));
	QPushButton* pbParamInit = new QPushButton(tr("Initialize"));
	connect(pbParamInit, SIGNAL(clicked()), SIGNAL(ParamInitSignal()));
	QPushButton* pbParamIter = new QPushButton(tr("Iterate"));
	connect(pbParamIter, SIGNAL(clicked()), SIGNAL(ParamIterSignal()));
	QPushButton* pbParamIterTillDone = new QPushButton(tr("Iterate Till Done"));
	connect(pbParamIterTillDone, SIGNAL(clicked()), SIGNAL(ParamITDSignal()));
	QPushButton* pbParamReset = new QPushButton(tr("Reset"));
	connect(pbParamReset, SIGNAL(clicked()), SIGNAL(ParamResetSignal()));

	QVBoxLayout* vbParamNonIter = new QVBoxLayout();
	vbParamNonIter->addWidget(pbParamRun);
	vbParamNonIter->addWidget(pbParamReset);

	QVBoxLayout* vbParamIter = new QVBoxLayout();
	vbParamIter->addWidget(pbParamInit);
	vbParamIter->addWidget(pbParamIter);
	vbParamIter->addWidget(pbParamIterTillDone);
	vbParamIter->addWidget(pbParamReset);

	QVBoxLayout* vbParam = new QVBoxLayout();
	vbParam->addWidget(lParamViewMode);
	if(has_boundary) vbParam->addWidget(cbParamPlanarViewMode);
	else
	{
		vbParam->addWidget(cbParamSphericalViewMode);
		vbParam->addWidget(lParamInitMethod);
		vbParam->addWidget(cbParamSphericalInitMethod);
	}
	vbParam->addWidget(lParamSolver);
	if (has_boundary) vbParam->addWidget(cbParamPlanarSolver);
	else vbParam->addWidget(cbParamSphericalSolver);
	vbParam->addWidget(lParamOperations);
	vbParam->addItem(vbParamIter);
	vbParam->addStretch();
	QWidget* wParam = new QWidget();
	wParam->setLayout(vbParam);
	saParam = new QScrollArea();
	saParam->setFrameStyle(QFrame::NoFrame);
	saParam->setWidget(wParam);
	saParam->setWidgetResizable(true);

	twParam->setCurrentIndex(twParam->addTab(saParam, "parameterization"));
}
