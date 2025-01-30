using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class MainPage : ContentPage
{
    public MainPage()
    {
        InitializeComponent();
        BindingContext = new MainPageViewModel();
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();

        if (BindingContext is MainPageViewModel viewModel)
        {
            viewModel.RefreshItems();
        }
    }
}
