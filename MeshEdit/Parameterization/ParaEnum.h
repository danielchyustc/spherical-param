#pragma once

enum WeightType {
	kEqual,
	kCotangent
};

enum BoundShape {
	kCircle,
	kSquare,
	kFree
};

enum ViewMode {
	kMesh,
	kSphere,
	kPlane
};

enum InitMethod {
	kMeshVoronoi,
	kHierCluster
};

enum PlanarMethod {
	kTutteEmbedding,
	kASAP,
	kARAP,
	kGD,
	kAQP,
	kAKVF,
	kSLIM,
	kCM
};

enum SphericalMethod {
	kGaussMap,
	kProjTutte,
	kSBE,
	kSCE,
	kCdVB,
	kCdVC,
	kSARAP,
	kSGD,
	kSAQP,
	kSSLIM
};

